/*
 * Project: Crankshaft
 * This file is part of Crankshaft project.
 * Copyright (C) 2026 OpenCarDev Team
 */


#include "HostapdWirelessNetworkManager.h"

#include <QDBusConnection>
#include <QDBusInterface>
#include <QDBusMessage>
#include <QDBusObjectPath>
#include <QDBusReply>
#include <QDBusVariant>
#include <QDeadlineTimer>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QThread>

#include "../logging/Logger.h"

namespace {

constexpr const char* kEnvDir = "/run/crankshaft";
constexpr const char* kEnvFile = "/run/crankshaft/aa-hotspot.env";
constexpr const char* kHelper = "/usr/lib/crankshaft/crankshaft-aa-hotspot";
constexpr const char* kIface = "wlan0";
constexpr const char* kAddress = "10.42.0.1";

QString readSysfs(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return {};
  }
  return QString::fromLatin1(file.readAll()).trimmed();
}

QString sanitise(QString value) {
  value.remove(QLatin1Char('\n'));
  value.remove(QLatin1Char('\r'));
  return value;
}

QDBusMessage callSystemd(const QString& method, const QVariantList& args) {
  QDBusMessage msg = QDBusMessage::createMethodCall(
      QStringLiteral("org.freedesktop.systemd1"), QStringLiteral("/org/freedesktop/systemd1"),
      QStringLiteral("org.freedesktop.systemd1.Manager"), method);
  msg.setArguments(args);
  return QDBusConnection::systemBus().call(msg, QDBus::Block, 10000);
}

}  // namespace

bool HostapdWirelessNetworkManager::isAvailable() {
  if (!QFileInfo(QString::fromLatin1(kHelper)).isExecutable()) {
    return false;
  }
  for (const char* dir : {"/usr/lib/systemd/system/", "/lib/systemd/system/",
                          "/etc/systemd/system/"}) {
    if (QFileInfo::exists(QString::fromLatin1(dir) + QString::fromLatin1(kUnitName))) {
      return true;
    }
  }
  return false;
}

bool HostapdWirelessNetworkManager::startHotspot(const HotspotConfig& config) {
  if (config.ssid.isEmpty()) {
    Logger::instance().warning("[Hotspot] Cannot start hotspot: SSID is empty");
    return false;
  }
  const bool open = config.security == SecurityMode::Open || config.password.isEmpty();
  if (!open && config.password.size() < 8) {
    Logger::instance().warning("[Hotspot] Cannot start hotspot: WPA2 password needs 8+ characters");
    return false;
  }

  QDir().mkpath(QString::fromLatin1(kEnvDir));
  QFile env(QString::fromLatin1(kEnvFile));
  if (!env.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    Logger::instance().warning(QString("[Hotspot] Cannot write %1: %2")
                                   .arg(QString::fromLatin1(kEnvFile), env.errorString()));
    return false;
  }
  env.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner);
  env.write(QStringLiteral("SSID=%1\nPSK=%2\nCHANNEL=%3\n")
                .arg(sanitise(config.ssid), open ? QString() : sanitise(config.password))
                .arg(config.channel)
                .toUtf8());
  env.close();

  QDBusReply<QDBusObjectPath> job = callSystemd(
      QStringLiteral("RestartUnit"), {QString::fromLatin1(kUnitName), QStringLiteral("replace")});
  if (!job.isValid()) {
    Logger::instance().warning(QString("[Hotspot] Could not start %1: %2")
                                   .arg(QString::fromLatin1(kUnitName), job.error().message()));
    return false;
  }

  // hostapd normally reports AP-ENABLED within a second. Wait briefly so the
  // first Bluetooth handshake doesn't send the phone to a hotspot that isn't up.
  QDeadlineTimer deadline(6000);
  bool up = false;
  while (!deadline.hasExpired()) {
    if (readSysfs(QStringLiteral("/sys/class/net/%1/operstate").arg(QString::fromLatin1(kIface))) ==
            QStringLiteral("up") &&
        unitActive()) {
      up = true;
      break;
    }
    QThread::msleep(200);
  }

  m_started = true;
  m_ssid = config.ssid;
  m_security = open ? SecurityMode::Open : SecurityMode::Wpa2Psk;
  Logger::instance().info(QString("[Hotspot] hostapd hotspot \"%1\" %2 (%3, %4)")
                              .arg(config.ssid,
                                   up ? QStringLiteral("started") : QStringLiteral("starting"),
                                   open ? QStringLiteral("open") : QStringLiteral("WPA2-PSK"),
                                   QString::fromLatin1(kAddress)));
  return true;
}

void HostapdWirelessNetworkManager::stopHotspot() {
  if (!m_started && !unitActive()) {
    return;
  }
  QDBusReply<QDBusObjectPath> job = callSystemd(
      QStringLiteral("StopUnit"), {QString::fromLatin1(kUnitName), QStringLiteral("replace")});
  if (!job.isValid()) {
    Logger::instance().warning(QString("[Hotspot] Could not stop %1: %2")
                                   .arg(QString::fromLatin1(kUnitName), job.error().message()));
  }
  m_started = false;
  Logger::instance().info("[Hotspot] hostapd hotspot stopped");
}

bool HostapdWirelessNetworkManager::unitActive() const {
  QDBusReply<QDBusObjectPath> unitPath =
      callSystemd(QStringLiteral("GetUnit"), {QString::fromLatin1(kUnitName)});
  if (!unitPath.isValid()) {
    return false;  // not loaded = not running
  }
  QDBusInterface props(QStringLiteral("org.freedesktop.systemd1"), unitPath.value().path(),
                       QStringLiteral("org.freedesktop.DBus.Properties"),
                       QDBusConnection::systemBus());
  QDBusReply<QDBusVariant> state = props.call(QStringLiteral("Get"),
                                              QStringLiteral("org.freedesktop.systemd1.Unit"),
                                              QStringLiteral("ActiveState"));
  return state.isValid() && state.value().variant().toString() == QStringLiteral("active");
}

IWirelessNetworkManager::HotspotStatus HostapdWirelessNetworkManager::getHotspotStatus() const {
  HotspotStatus status;
  // Only report a hotspot this process configured: one left running by an
  // earlier core has a different random password.
  if (!m_started || !unitActive()) {
    return status;
  }
  status.active = true;
  status.ssid = m_ssid;
  status.bssid =
      readSysfs(QStringLiteral("/sys/class/net/%1/address").arg(QString::fromLatin1(kIface)))
          .toUpper();
  status.ipAddress = QString::fromLatin1(kAddress);
  status.security = m_security;
  return status;
}
