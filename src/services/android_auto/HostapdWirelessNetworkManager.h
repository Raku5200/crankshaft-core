/*
 * Project: Crankshaft
 * This file is part of Crankshaft project.
 * Copyright (C) 2026 OpenCarDev Team
 */


#pragma once

#include <QString>

#include "IWirelessNetworkManager.h"

/**
 * Wireless AA hotspot run by hostapd (crankshaft-aa-hotspot.service).
 *
 * NetworkManager always offers WPA-PSK-SHA256 on its hotspots when
 * wpa_supplicant supports PMF, and the Pi's brcmfmac can't complete that
 * handshake. hostapd offers plain WPA2-PSK only. This class writes the SSID
 * and password to /run/crankshaft/aa-hotspot.env and (re)starts the unit
 * through systemd over D-Bus (allowed by a polkit rule).
 */
class HostapdWirelessNetworkManager final : public IWirelessNetworkManager {
 public:
  static constexpr const char* kUnitName = "crankshaft-aa-hotspot.service";

  HostapdWirelessNetworkManager() = default;

  /// True if the hotspot unit and helper are installed on this system.
  static bool isAvailable();

  bool startHotspot(const HotspotConfig& config) override;
  void stopHotspot() override;
  HotspotStatus getHotspotStatus() const override;

 private:
  bool unitActive() const;

  bool m_started{false};
  QString m_ssid;
  SecurityMode m_security{SecurityMode::Wpa2Psk};
};
