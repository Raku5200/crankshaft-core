/*
 * Project: Crankshaft
 * This file is part of Crankshaft project.
 * Copyright (C) 2025 OpenCarDev Team
 *
 *  Crankshaft is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Crankshaft is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Crankshaft. If not, see <http://www.gnu.org/licenses/>.
 */

#include "CoreApplicationRunner.h"

#include <QByteArray>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QStringList>
#include <QTimer>

#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "services/config/ConfigService.h"
#include "services/eventbus/EventBus.h"
#include "services/logging/Logger.h"
#include "services/profile/ProfileManager.h"
#include "services/service_manager/ServiceManager.h"
#include "services/websocket/WebSocketServer.h"
#include "services/ibus/IbusService.h"

namespace {

// Logs when the main (Qt) thread stops running its event loop for more than
// 2 s, with the kernel function it is waiting in, e.g. a USB ioctl or a lock.
// Diagnoses start-up/connection stalls on the Pi; costs nothing otherwise.
class MainThreadStallWatchdog {
 public:
  void start(QCoreApplication* app) {
    m_mainTid = static_cast<long>(::syscall(SYS_gettid));
    m_beatMs.store(nowMs());
    auto* beat = new QTimer(app);
    QObject::connect(beat, &QTimer::timeout, [this]() { m_beatMs.store(nowMs()); });
    beat->start(250);
    std::thread([this]() { run(); }).detach();
  }

 private:
  static auto nowMs() -> long long {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  auto readProc(const char* leaf) const -> std::string {
    std::ifstream in("/proc/self/task/" + std::to_string(m_mainTid) + "/" + leaf);
    std::string line;
    std::getline(in, line);
    return line;
  }
  void run() {
    bool stalled = false;
    long long stallStart = 0;
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const long long now = nowMs();
      const long long since = now - m_beatMs.load();
      if (!stalled && since > 2000) {
        stalled = true;
        stallStart = m_beatMs.load();
        std::fprintf(stderr, "[Watchdog] main thread stalled for %lld ms, waiting in %s (syscall %s)\n",
                     since, readProc("wchan").c_str(), readProc("syscall").c_str());
      } else if (stalled && since > 2000 && (since / 5000) != ((since - 500) / 5000)) {
        std::fprintf(stderr, "[Watchdog] main thread still stalled (%lld ms), waiting in %s\n", since,
                     readProc("wchan").c_str());
      } else if (stalled && since <= 2000) {
        stalled = false;
        std::fprintf(stderr, "[Watchdog] main thread resumed after %lld ms\n",
                     m_beatMs.load() - stallStart);
      }
    }
  }

  long m_mainTid{0};
  std::atomic<long long> m_beatMs{0};
};
bool isVerboseUsbFlagPresent(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == std::string("--verbose-usb") || arg == std::string("-v")) {
      return true;
    }
  }
  return false;
}

bool initializeLoggerFromConfig() {
  QString logLevel =
      ConfigService::instance().get("core.logging.level", "info").toString().toLower();
  QString logFilePath = ConfigService::instance().get("core.logging.file_path", "").toString();

  if (logFilePath.isEmpty()) {
    logFilePath = ConfigService::instance().get("core.logging.file", "").toString();
  }
  if (logFilePath.isEmpty()) {
#ifdef Q_OS_WIN
    logFilePath = "%LOCALAPPDATA%/Crankshaft/crankshaft.log";
#else
    logFilePath = "/var/log/crankshaft/crankshaft.log";
#endif
  }

#ifndef Q_OS_WIN
  if (logFilePath.startsWith("/var/log")) {
    QDir logDir;
    if (!logDir.exists("/var/log/crankshaft")) {
      if (!logDir.mkpath("/var/log/crankshaft")) {
        Logger::instance().warning("Failed to create /var/log/crankshaft directory");
      }
    }
  }
#endif

  Logger::Level level = Logger::Level::Info;
  if (logLevel == "debug") {
    level = Logger::Level::Debug;
  } else if (logLevel == "warning") {
    level = Logger::Level::Warning;
  } else if (logLevel == "error") {
    level = Logger::Level::Error;
  } else if (logLevel == "fatal") {
    level = Logger::Level::Fatal;
  }

  // The journal already keeps everything (and Save logs reads it). A second
  // copy in /var/log/crankshaft doubled the SD card writes, so it is only
  // written when core.logging.to_file is true.
  if (!ConfigService::instance().get("core.logging.to_file", false).toBool()) {
    logFilePath.clear();
  }

  Logger::instance().setLevel(level);
  Logger::instance().setLogFile(logFilePath);
  Logger::instance().info(
      QString("Logging configured: level=%1, file_path=%2")
          .arg(logLevel, logFilePath.isEmpty() ? QStringLiteral("<journal only>") : logFilePath));

  return true;
}
}  // namespace

int runCoreApplication(int argc, char* argv[], const CoreBuildInfo& buildInfo) {
  QElapsedTimer startupTimer;
  startupTimer.start();

  QCoreApplication app(argc, argv);
  static MainThreadStallWatchdog stallWatchdog;
  stallWatchdog.start(&app);
  app.setApplicationName("Crankshaft");
  app.setApplicationVersion("1.0.0");
  app.setOrganizationName("OpenCarDev");

  bool verboseUsbArgPresent = isVerboseUsbFlagPresent(argc, argv);
  if (verboseUsbArgPresent) {
    qputenv("AASDK_VERBOSE_USB", "1");
  }

  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == std::string("--verbose-usb") || arg == std::string("-v")) {
      verboseUsbArgPresent = true;
      break;
    }
  }

  QCommandLineParser parser;
  parser.setApplicationDescription("Crankshaft Automotive Infotainment Core");
  parser.addHelpOption();
  parser.addVersionOption();

  QCommandLineOption portOption(QStringList() << "p" << "port", "WebSocket server port", "port",
                                "8080");
  parser.addOption(portOption);

  QCommandLineOption configOption(QStringList() << "c" << "config", "Configuration file path",
                                  "config", "../config/crankshaft.json");
  parser.addOption(configOption);

  QCommandLineOption verboseUsbOption(
      QStringList() << "verbose-usb",
      "Enable verbose AASDK USB logging (or use env AASDK_VERBOSE_USB=1)");
  parser.addOption(verboseUsbOption);

  parser.process(app);

  bool verboseUsb = false;
  const QByteArray verboseUsbEnv = qgetenv("AASDK_VERBOSE_USB");
  if (!verboseUsbEnv.isEmpty()) {
    QByteArray lower = verboseUsbEnv.toLower();
    verboseUsb = (lower == "1" || lower == "true" || lower == "yes");
  }
  if (!verboseUsb && (parser.isSet(verboseUsbOption) || verboseUsbArgPresent)) {
    verboseUsb = true;
  }

  if (verboseUsb) {
    Logger::instance().info("AASDK verbose USB logging requested");
  }

  Logger::instance().setLevel(Logger::Level::Info);
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: Starting Crankshaft Core...").arg(startupTimer.elapsed()));

  Logger::instance().info(
      QString("Build timestamp: %1, commit(short): %2, commit(long): %3, branch: %4")
          .arg(QString::fromUtf8(buildInfo.timestamp))
          .arg(QString::fromUtf8(buildInfo.commitShort))
          .arg(QString::fromUtf8(buildInfo.commitLong))
          .arg(QString::fromUtf8(buildInfo.branch)));

  const QString cliConfigPath = parser.value(configOption).trimmed();
  const QString envConfigPath = QString::fromUtf8(qgetenv("CRANKSHAFT_CONFIG")).trimmed();

  QStringList configCandidates;
  if (!envConfigPath.isEmpty()) {
    configCandidates << envConfigPath;
  }
  if (!cliConfigPath.isEmpty()) {
    configCandidates << cliConfigPath;
  }
  configCandidates << "../config/crankshaft.json";
  configCandidates << "/etc/crankshaft/crankshaft.json";
  configCandidates.removeDuplicates();

  bool configLoaded = false;
  QString loadedConfigPath;
  QStringList attemptedConfigPaths;
  QStringList candidateLoadFailures;

  for (const QString& candidate : configCandidates) {
    attemptedConfigPaths << candidate;

    if (!QFileInfo::exists(candidate)) {
      continue;
    }

    if (ConfigService::instance().load(candidate)) {
      configLoaded = true;
      loadedConfigPath = candidate;
      break;
    }

    const QString loadError = ConfigService::instance().lastLoadError();
    candidateLoadFailures <<
        QString("%1 => %2")
            .arg(candidate,
                 loadError.isEmpty() ? QStringLiteral("load failed (no detail)") : loadError);
  }

  if (!configLoaded) {
    if (!candidateLoadFailures.isEmpty()) {
      Logger::instance().error(
          QString("Configuration load failures: %1").arg(candidateLoadFailures.join(" | ")));
    }
    Logger::instance().warning(QString("Using default configuration; attempted paths: %1")
                                   .arg(attemptedConfigPaths.join(", ")));
    Logger::instance().warning(
        "[STARTUP] CONFIG_FALLBACK_DEFAULTS=true (continuing with built-in defaults)");
  } else {
    Logger::instance().info(QString("Using configuration from %1").arg(loadedConfigPath));
  }
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: Configuration loaded").arg(startupTimer.elapsed()));

  initializeLoggerFromConfig();
  Logger::instance().info(QString("[STARTUP] %1ms elapsed: Logger configured from config file")
                              .arg(startupTimer.elapsed()));

  quint16 port = parser.value(portOption).toUInt();
  if (port == 0) {
    port = ConfigService::instance().get("core.websocket.port", 8080).toUInt();
  }

  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: Initialising core services...").arg(startupTimer.elapsed()));
  EventBus::instance();
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: Event bus initialised").arg(startupTimer.elapsed()));

  Logger::instance().info(QString("[STARTUP] %1ms elapsed: Initialising ProfileManager...")
                              .arg(startupTimer.elapsed()));
  QString profileConfigDir = ConfigService::instance()
                                 .get("core.profile.configDir", "/etc/crankshaft/profiles")
                                 .toString();
  ProfileManager profileManager(profileConfigDir);

  if (!profileManager.loadProfiles()) {
    Logger::instance().warning("Failed to load profiles, using default profiles");
  }
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: ProfileManager initialised").arg(startupTimer.elapsed()));

  Logger::instance().info(QString("[STARTUP] %1ms elapsed: Initialising ServiceManager...")
                              .arg(startupTimer.elapsed()));
  ServiceManager serviceManager(&profileManager);
  if (!serviceManager.startAllServices()) {
    Logger::instance().warning("No services started successfully from active profile");
  }
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: ServiceManager initialised").arg(startupTimer.elapsed()));

  Logger::instance().info(QString("[STARTUP] %1ms elapsed: Starting WebSocket server on port %2...")
                              .arg(startupTimer.elapsed())
                              .arg(port));
  WebSocketServer webSocketService(port);
  if (!webSocketService.isListening()) {
    Logger::instance().error("Failed to start WebSocket service");
    return 1;
  }
  webSocketService.setServiceManager(&serviceManager);
  webSocketService.initializeServiceConnections();

  // Car IBus (Resler or other USB interface); idle until the interface appears.
  crankshaft::ibus::IbusService ibusService;
  webSocketService.setIbusService(&ibusService);
  ibusService.start();
  Logger::instance().info(
      QString("[STARTUP] %1ms elapsed: WebSocket service started").arg(startupTimer.elapsed()));

  auto& eventBus = EventBus::instance();
  QObject::connect(&eventBus, &EventBus::messagePublished, &app,
                   [](const QString& topic, const QVariantMap&) {
                     if (topic == "android_auto/device_connected") {
                       Logger::instance().info("Android Auto connected");
                     } else if (topic == "android_auto/device_disconnected") {
                       Logger::instance().info("Android Auto disconnected");
                     }
                   });
  Logger::instance().info(QString("[STARTUP] %1ms elapsed: Crankshaft Core initialisation complete")
                              .arg(startupTimer.elapsed()));

  return app.exec();
}
