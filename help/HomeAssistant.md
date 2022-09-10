# Mowgli HA Integration: #

(via om_ha_bridge.py)
### Configure Mowgli Bridge

```
cp ~/MowgliRover/src/mowgli/scripts/bridge.conf.example ~/MowgliRover/src/mowgli/scripts/bridge.conf
```

Edit bridge.conf and setup your HA MQTT details

### MQTT Mowgli ###
```
sensor:
  # MOWGLI
  - name: "Mowgli Current Behaviour"
    platform: mqtt
    state_topic: "mowgli/current_behaviour"
  - name: "Mowgli Charge Current"
    platform: mqtt
    state_topic: "mowgli/i_charge"
    unit_of_measurement: "A"
  - name: "Mowgli Charge Voltage"
    platform: mqtt
    state_topic: "mowgli/v_charge"
    unit_of_measurement: "V"
  - name: "Mowgli Battery Voltage"
    platform: mqtt
    state_topic: "mowgli/v_battery"
    unit_of_measurement: "V"
  - name: "Mowgli is Charging ?"
    platform: mqtt
    state_topic: "mowgli/is_charging"
  - name: "Mowgli GPS Tracker"
    platform: mqtt
    state_topic: "mowgli/is_charging"
  - name: "Mowgli GPS Tracker / Latitude"
    platform: mqtt
    state_topic: "mowgli/lat"
  - name: "Mowgli GPS Tracker / Longitude"
    platform: mqtt
    state_topic: "mowgli/lon"
```

### Device Tracker ###

Add this to your known_devices.yaml

```
mowgli_gps_tracker:
  name: mowgli_gps_tracker
  mac:
  icon: mdi:robot-mower-outline
  picture:
  track: true
```

### Automation ###

Add this as an automation that will use the device_tracker.see service to update
the GPS location of "mowgli_gps_tracker"

```
- id: "1660585701592"
  alias: Mowgli GPS Tracker
  description: ""
  trigger:
    - platform: state
      entity_id: sensor.mowgli_gps_tracker_latitude, sensor.mowgli_gps_tracker_longitude
  condition: []
  action:
    - service: device_tracker.see
      data_template:
        dev_id: mowgli_gps_tracker
        battery: "{{ states('sensor.mowgli_battery_voltage') }}"
        gps_accuracy: 0.05
        gps:
          - "{{ states('sensor.mowgli_gps_tracker_latitude') }}"
          - "{{ states('sensor.mowgli_gps_tracker_longitude') }}"
```

### LoveLace GPS Card ###

```
- title: Mowgli Tracker
    path: mowgli-gps
    icon: mdi:map
    type: panel
    badges: []
    cards:
      - type: horizontal-stack
        cards:
          - type: map
            entities:
              - device_tracker.mowgli_gps_tracker
            title: Mowgli GPS Tracker
            hours_to_show: 6
            dark_mode: false
```

### LoveLace Mowgli Command / Status ###

```
  - title: Mowgli
    path: mowgli
    icon: mdi:robot-mower
    badges: []
    cards:
      - type: vertical-stack
        cards:
          - type: markdown
            title: Current Behaviour
            content: '{{ states(''sensor.mowgli_current_behaviour'') }}'
          - type: horizontal-stack
            cards:
              - type: button
                tap_action:
                  action: call-service
                  service: mqtt.publish
                  service_data:
                    payload: command_start
                    qos: '1'
                    topic: mowgli/hactrl
                  target: {}
                icon: mdi:play
                name: START
              - type: button
                tap_action:
                  action: call-service
                  service: mqtt.publish
                  service_data:
                    payload: command_home
                    qos: '1'
                    topic: mowgli/hactrl
                  target: {}
                icon: mdi:home
                name: HOME
          - type: horizontal-stack
            cards:
              - type: button
                tap_action:
                  action: call-service
                  service: mqtt.publish
                  service_data:
                    payload: command_s1
                    qos: '1'
                    topic: mowgli/hactrl
                  target: {}
                icon: mdi:pause
                name: S1
              - type: button
                tap_action:
                  action: call-service
                  service: mqtt.publish
                  service_data:
                    payload: command_s2
                    qos: '1'
                    topic: mowgli/hactrl
                  target: {}
                icon: mdi:stop
                name: S2
          - type: horizontal-stack
            cards:
              - type: gauge
                entity: sensor.mowgli_battery_voltage
                min: 0
                unit: V
                max: 30
              - type: gauge
                entity: sensor.mowgli_charge_voltage
                min: 0
                max: 32
                unit: V
          - type: horizontal-stack
            cards:
              - type: gauge
                entity: sensor.mowgli_charge_current
                min: 0
                max: 2
                unit: A
```


# Raspi CAM Stream for HA

## Enable Camera

Follow: https://blog.fearcat.in/a?ID=01800-ebec982b-7267-4650-9aa0-4b00d225f414

## Integrate Camera into HA
(taken from https://siytek.com/raspberry-pi-rtsp-to-home-assistant )

```
cd /home/ubuntu
sudo apt-get install cmake liblog4cpp5-dev libv4l-dev v4l2-ctl git -y
git clone https://github.com/mpromonet/v4l2rtspserver.git ; cd v4l2rtspserver/ ; cmake . ; make ; sudo make install
```

edit /lib/systemd/system/v4l2rtspserver.service
and replace the ExecStart line with

```
ExecStart=v4l2rtspserver -W 640 -H 480 -F 15 -P 8554 /dev/video0
```

enable and start service

```
sudo systemctl enable v4l2rtspserver
sudo systemctl start v4l2rtspserver
```

### HA config examples

Add to HA configuration, and restart

```
camera:
  - platform: ffmpeg
    name: RasPi4
    input: rtsp://<raspberry-pi-ip>:8554/unicast
```

Add a Picture Glance card and select the camera object

### Flipping camera output

I had todo both a vertical and horizontal for my camera

```
sudo v4l2-ctl --set-ctrl vertical_flip=1
sudo v4l2-ctl --set-ctrl horizontal_flip=1
```

You will also need to add that to /lib/systemd/system/v4l2rtspserver.service as it does not persist.

### Testing

Test accessing the rtsp stream with VLC by opening 

```
rtsp://<raspberry-pi-ip>:8554/unicast
```

With VLC redirection you can also save your Mowgli adventures ;-)

