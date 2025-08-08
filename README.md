# esp32-wxserv
Simple Weatherserver for sending APRS Weather Messages

## How?
Connect to it with: nc -d <ipaddress> 1432 <br/>

	and perhaps send immediately:<br/>
nc -d esp32-wxserv 1432 | kissutil -h aprs-hostname


