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

#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <cstring>
#include <initializer_list>

namespace {

// Writes a raw backtrace to stderr (captured by journald) when core crashes, then
// re-raises the signal so systemd still sees the crash and restarts the service.
// Frames are printed as "binary(+0xoffset)", which can be resolved afterwards
// with addr2line against the same build. Only async-signal-safe calls are used.
void crashHandler(int sig) {
  static const char header[] = "\n[CrashHandler] crankshaft-core received fatal signal ";
  ssize_t ignored = write(STDERR_FILENO, header, sizeof(header) - 1);
  const char* name = sig == SIGSEGV   ? "SIGSEGV"
                     : sig == SIGABRT ? "SIGABRT"
                     : sig == SIGBUS  ? "SIGBUS"
                     : sig == SIGFPE  ? "SIGFPE"
                     : sig == SIGILL  ? "SIGILL"
                                      : "?";
  ignored = write(STDERR_FILENO, name, strlen(name));
  static const char tail[] = ", backtrace:\n";
  ignored = write(STDERR_FILENO, tail, sizeof(tail) - 1);
  (void)ignored;

  void* frames[64];
  const int count = backtrace(frames, 64);
  backtrace_symbols_fd(frames, count, STDERR_FILENO);

  signal(sig, SIG_DFL);
  raise(sig);
}

void installCrashHandler() {
  // Load libgcc's unwinder now; backtrace() may allocate on first use.
  void* warmup[1];
  backtrace(warmup, 1);

  struct sigaction action {};
  action.sa_handler = crashHandler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESETHAND;
  for (int sig : {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL}) {
    sigaction(sig, &action, nullptr);
  }
}

}  // namespace

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Generated build info
const char CRANKSHAFT_BUILD_TIMESTAMP[] = TOSTRING(CRANKSHAFT_BUILD_TIMESTAMP_VALUE);
const char CRANKSHAFT_GIT_COMMIT_SHORT[] = TOSTRING(CRANKSHAFT_GIT_COMMIT_SHORT_VALUE);
const char CRANKSHAFT_GIT_COMMIT_LONG[] = TOSTRING(CRANKSHAFT_GIT_COMMIT_LONG_VALUE);
const char CRANKSHAFT_GIT_BRANCH[] = TOSTRING(CRANKSHAFT_GIT_BRANCH_VALUE);

int main(int argc, char* argv[]) {
  installCrashHandler();
  CoreBuildInfo buildInfo{CRANKSHAFT_BUILD_TIMESTAMP, CRANKSHAFT_GIT_COMMIT_SHORT,
                          CRANKSHAFT_GIT_COMMIT_LONG, CRANKSHAFT_GIT_BRANCH};
  return runCoreApplication(argc, argv, buildInfo);
}
