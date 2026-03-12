#create service for mqtt bridge
sudo nano /etc/systemd/system/mqtt-tcp-bridge.service

#paste in service created
[Unit]
Description=MQTT to TCP Bridge Service
After=network.target network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/mqtt_bridge.py
WorkingDirectory=/home/pi/
Restart=always
RestartSec=5
User=pi
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target

#Replace /home/pi/mqtt_bridge.py with the actual path to your Python script.

#create and start service
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable mqtt-tcp-bridge.service
sudo systemctl start mqtt-tcp-bridge.service

#check status of service
sudo systemctl status mqtt-tcp-bridge.service

#follow logs
journalctl -u mqtt-tcp-bridge.service -f

#make the python script sh executable
chmod +x /home/pi/mqtt_bridge.py

#list running services
systemctl list-units --type=service
systemctl list-units --type=service --state=running

