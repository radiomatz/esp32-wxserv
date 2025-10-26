# esp32-wxserv
Simple Weatherserver for sending APRS Weather Messages

## How?
<p>First change your mqtt-hostname, router-ssid, router-bssid (MAC Adress of router), WiFi-Password in the source, then upload to your esp32-devkit<br>
you can pause WiFi in the night in the source, if you need this.<br>

Connect to it with: <br/>
	`nc -d <ipaddress> 1432`

and perhaps send immediately:<br/>
	`nc -d esp32-wxserv 1432 | kissutil -h aprs-hostname`


