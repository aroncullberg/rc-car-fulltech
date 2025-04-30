# Factory settings

* GPS+Galileo+GLONASS+BeiDou+QZSS+SBAS
* 38400 baud, 8 bits, no parity bit, 1 stop bit.
* 1Hz navigation update rate.
* 1PPS time pulse
* NMEA version 4.11.
* Input messages: NMEA and UBX.
* Output messages: NMEA GGA, GLL, GSA, GSV, RMC, VTG and TXT.

___ 

### NMEA0183 ver4.11

| NMEA Message   | Description                                               | Enable |
|----------------|-----------------------------------------------------------|--------|
| $Talker ID+GGA | Time, position, and fix related data                      | Y      |
| $Talker ID+GLL | Position data: position fix, time of position, and status | Y      |
| $Talker ID+GSA | GNSS DOP and active satellites                            | Y      |
| $Talker ID+GSV | Number of SVs in view, PRN, elevation, azimuth, and SNR   | Y      |
| $Talker ID+RMC | Position, Velocity, and Time                              | Y      |
| $Talker ID+VTG | Course over ground and ground speed                       | Y      |

| ZDA GNS DTM GBS GST GRS etc. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | N&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------|

---

### Talker ID's
| Talker ID | Description                                                     |
|-----------|-----------------------------------------------------------------|
| GP        | GPS (Global Positioning System) only                            |
| GL        | GLONASS (Global Navigation Satellite System) only               |
| GA        | Galileo (Global Navigation Satellite System)                    |
| BD        | BeiDou (Chinese Satellite Navigation System)                    |
| QQ        | QZSS                                                            |
| GN        | Global Navigation Satellite System (GPS+GLONASS+Galileo+BeiDou) |
