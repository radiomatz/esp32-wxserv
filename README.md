# esp32-wxserv
Simple Weatherserver for sending APRS Weather Messages

## How?
<p>First change your mqtt-hostname, router-ssid, router-bssid (MAC Adress of router), WiFi-Password in the source, then upload to your esp32-devkit<br>

Connect to it with: <br/>
	`nc <ipaddress> 1432`

and perhaps send immediately:<br/>
	`nc esp32-wxserv 1432 | kissutil -h aprs-hostname`

nc - netcat - depending on your version, you may use the parameter -d for not reading input to send it.


