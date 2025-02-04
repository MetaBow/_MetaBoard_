## Install dependencies

Python 3 needs to be installed. Whatever version is compatible with the latest version of bleak.

```
pip install -r requirements.txt
```

## Finding Your Device

A few options can be provided to the Python program, like `--scan` which will
scan for BLE devices and list their addresses and names for you.

```
python ble_data_bridge/__init__.py --scan
```

To connect to a device
```
python ble_data_bridge/__init__.py --name "metabow"
```

Or you can provide the address, which is a bit faster because no scan is required:
```
python ble_data_bridge/__init__.py --address "xx:xx:xx:xx:xx:xx"
```

The OSC bridge is running on localhost:5005 sending quaternion data as a float array I,J,K,Real (X,Y,Z,W)
