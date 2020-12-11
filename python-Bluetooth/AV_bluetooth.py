import asyncio
import bleak
import os
import subprocess

# // 0x180F -> Battery Service UUID (128-bit UUID: 0000180F-0000-1000-8000-00805F9B34FB)
# // 0x2A00 -> Device Name Characteristic UUID (128-bit UUID: 00002A00-0000-1000-8000-00805F9B34FB)


# async def run():
#     devices = await discover()
#     for d in devices:
#         print(d)

async def run(address, loop):
    async with BleakClient(address, loop=loop) as client:
        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

# def run(address, loop):
#     with BleakClient(address, loop=loop) as client:
#         model_number = client.read_gatt_char(MODEL_NBR_UUID)
#         print("Model Number: {0}".format("".join(map(chr, model_number))))

def AV_ble_UUID128bits(uuid16bits_p):

    return ("0000" + str(hex(uuid16bits_p)).replace("0x", "") + "-0000-1000-8000-00805f9b34fb")

def AV_ble_isDeviceExist(address_p="00:81:f9:2a:c4:32"):

    x_l = AV_ble_discover()

    for i_l in x_l:

        if i_l.address == address_p:
            return True

    return False

def AV_ble_discover(search_txt_p=None):
    loop_l = asyncio.get_event_loop()

    devices_l = loop_l.run_until_complete(bleak.discover())

    if search_txt_p is None:
        return devices_l
    else:
        selected_devices_l = []
        
        for d_l in devices_l:
            if search_txt_p in str(d_l):
                selected_devices_l.append(d_l)

        return selected_devices_l

def AV_ble_client(address_p="3C:71:BF:FD:31:EA"):
    return bleak.BleakClient(address_p)

def AV_ble_connect(client_p):
    loop_l = asyncio.get_event_loop()

    result_l = loop_l.run_until_complete(client_p.connect())

    return result_l

def AV_ble_is_connected(client_p):
    loop_l = asyncio.get_event_loop()

    return loop_l.run_until_complete(client_p.is_connected())

def AV_ble_read_gatt_char(client_p, uuid128bits_p):
    loop_l = asyncio.get_event_loop()

    return loop_l.run_until_complete(client_p.read_gatt_char(uuid128bits_p))

def AV_ble_disconnect(client_p):
    loop_l = asyncio.get_event_loop()

    loop_l.run_until_complete(client_p.disconnect())


def AV_ble_win10_discover():
    """
    This funciton only works in Windows 10. It requires Bluetooth Command Line Tools (http://bluetoothinstaller.com/bluetooth-command-line-tools/).
    Make sure that Bluetooth Command Line Tools are in the path.
    """


    cmd_l = "btdiscovery -s"

    subprocess.call(cmd_l, shell=True)

def AV_ble_win10_pair_COM(address_p="00:81:F9:2A:C4:32", pin_p="0000", friendlyName_p="MindWave Mobile"):
    """
    This funciton only works in Windows 10. It requires Bluetooth Command Line Tools (http://bluetoothinstaller.com/bluetooth-command-line-tools/).
    Make sure that Bluetooth Command Line Tools are in the path.
    """

    if AV_ble_isDeviceExist(address_p):

        print("[AV_ble_pair_win10]: Pairing to ", address_p)

        cmd_l = "btcom -b\"" + address_p + "\" -c"
        ret_l = subprocess.call(cmd_l, shell=True)
        if  ret_l == 0:

            cmd_l = "btpair -p" + pin_p + " -n\"" + friendlyName_p + "\""
            ret_l = subprocess.call(cmd_l, shell=True)
            if  ret_l == 0:
                print("[AV_ble_pair_win10]: Paired to ", address_p)


                return AV_ble_win10_getCOM(address_p)

            else:

                return None

        else:
            print("[AV_ble_pair_win10]: Fail to pair to ", address_p, " with error ", ret_l)

            return None

    else:
        print("[AV_ble_win10_pair]: Device does not exist.")

        return None

def AV_ble_win10_unpair(address_p=None):
    """
    This funciton only works in Windows 10. It requires Bluetooth Command Line Tools (http://bluetoothinstaller.com/bluetooth-command-line-tools/).
    Make sure that Bluetooth Command Line Tools are in the path.
    """

    if address_p is None:
        cmd_l = "btpair -u"

        print("[AV_ble_pair_win10]: Unpairing all devices remembered by Bluetooth Command Line Tools")
    else:
        cmd_l = "btpair -b\"" + address_p + "\" -u"

        print("[AV_ble_pair_win10]: Unpairing " + address_p)

    ret_l = subprocess.call(cmd_l, shell=True)
    if  ret_l == 0:
        print("[AV_ble_pair_win10]: Unpaired success")

        return True
    else:
        print("[AV_ble_pair_win10]: Unpaired fail with error ", ret_l)

        return False

def AV_ble_win10_getCOM(address_p="00:81:F9:2A:C4:32"):

    import winreg
    import serial
    import time

    class BluetoothSpp:
        key_bthenum = r"SYSTEM\CurrentControlSet\Enum\BTHENUM"

        def get_spp_com_port(self, bt_mac_addr):
            bt_mac_addr = bt_mac_addr.replace(':', '').upper()
            for i in self.gen_enum_key('', 'LOCALMFG'):
                for j in self.gen_enum_key(i, bt_mac_addr):
                    DEBUG_PORT = j.split("_")[1]
                    if DEBUG_PORT in j:
                        subkey = self.key_bthenum+'\\'+ i+'\\'+j
                        port = self.get_reg_data(subkey, 'FriendlyName')
                        assert('Standard Serial over Bluetooth link' in port[0])
                        items = port[0].split()
                        port = items[5][1:-1]
                        return port

        def gen_enum_key(self, subkey, search_str):
            hKey = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, self.key_bthenum + '\\' + subkey)
            print(subkey,search_str)
            try:
                i = 0
                while True:
                    output = winreg.EnumKey(hKey, i)
                    if search_str in output:
                        yield output
                    i += 1

            except:
                pass

            winreg.CloseKey(hKey)

        def get_reg_data(self, subkey, name):
            hKey = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
                                subkey)
            output = winreg.QueryValueEx(hKey, name) 
            winreg.CloseKey(hKey)
            return output

    return (BluetoothSpp().get_spp_com_port(address_p))

if __name__ == "__main__":

    # print(AV_ble_isDeviceExist("40:2B:48:FB:42:55"))
    # x = AV_ble_discover()

    # for i in x:
    #     print(i.address)


    # print(AV_ble_win10_pair())

    print(AV_ble_win10_getCOM())

    # AV_ble_win10_unpair("00:81:F9:2A:C4:32")

    exit()

    devinfos_char_mod_num_str_uuid = 0x2A24 
    devinfos_char_serial_num_str_uuid     = 0x2A25
    battery_char_level_uuid = 0x2A19;


    devices_l = AV_ble_discover("i-Aware")

    for d in devices_l:
        print(d)


    client_l = AV_ble_client(address_p="3C:71:BF:FD:31:EA")

    if AV_ble_connect(client_l):
        print("connect")

        print(AV_ble_is_connected(client_l))

        ret_l = AV_ble_read_gatt_char(client_l, uuid128bits_p=AV_ble_UUID128bits(uuid16bits_p=devinfos_char_serial_num_str_uuid))
        print(ret_l)

        AV_ble_disconnect(client_l)        
    else:
        print("cannot connect")

    exit()

    # address = "3C:71:BF:FD:31:EA"
    # MODEL_NBR_UUID = "00002a24-0000-1000-8000-00805f9b34fb"

    # devinfos_char_mod_num_str_uuid = 0x2A24 
    # battery_char_level_uuid = 0x2A19;

    # uuid_l = _16bits_to_128bitsUUID(uuid16bits_p=battery_char_level_uuid)

    # loop = asyncio.get_event_loop()
    # # # # loop.run_until_complete(run())

    # # loop.run_until_complete(run(address, loop))
    # # loop.run_until_complete(connectBLE(address, loop))

    # # connectBLE(address, loop)

    # # run(address, loop)
    

    # # ble_client_l = BleakClient(address)

    # # model_number = await ble_client_l.read_gatt_char(MODEL_NBR_UUID)
    # # print(model_number)

    # ble_client_l = BleakClient(address)

    # result = loop.run_until_complete(ble_client_l.connect())

    # if result:
    #     print("connect")

    #     srvs_l = loop.run_until_complete(ble_client_l.get_services())

    #     # print(srvs_l)
    #     for srv in srvs_l:
    #         print(srv)
    # else:
    #     print("disconnect")

    
    # result = loop.run_until_complete(ble_client_l.disconnect())

    # # # Initialize the BLE system.  MUST be called before other BLE calls!
    # # ble.initialize()
     
    # # # Start the mainloop to process BLE events, and run the provided function in
    # # # a background thread.  When the provided main function stops running, returns
    # # # an integer status code, or throws an error the program will exit.
    # # ble.run_mainloop_with(main)
