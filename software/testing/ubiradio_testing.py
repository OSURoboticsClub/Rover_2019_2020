import paramiko
import json
import time

# ath0      21 channels in total; available frequencies :
#           Channel 01 : 2.412 GHz
#           Channel 31 : 2.414 GHz
#           Channel 02 : 2.417 GHz
#           Channel 32 : 2.419 GHz
#           Channel 03 : 2.422 GHz
#           Channel 33 : 2.424 GHz
#           Channel 04 : 2.427 GHz
#           Channel 34 : 2.429 GHz
#           Channel 05 : 2.432 GHz
#           Channel 35 : 2.434 GHz
#           Channel 06 : 2.437 GHz
#           Channel 36 : 2.439 GHz
#           Channel 07 : 2.442 GHz
#           Channel 37 : 2.444 GHz
#           Channel 08 : 2.447 GHz
#           Channel 38 : 2.449 GHz
#           Channel 09 : 2.452 GHz
#           Channel 39 : 2.454 GHz
#           Channel 10 : 2.457 GHz
#           Channel 40 : 2.459 GHz
#           Channel 11 : 2.462 GHz
#           Current Frequency:2.417 GHz (Channel 2)

# Sets: iwconfig ath0 channel 01
# Gets: iwlist ath0 channel
# NOTE
# Only the access point has to get changed the station (client) will automatically choose the new freq

channel = 3

get_general_info = "wstalist"
get_wireless_info = "iwlist ath0 channel"
set_wireless_frequency = "iwconfig ath0 channel " + "%02d" % channel  # iwconfig ath0 freq 2.456G

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

# Before anyone complains, I'm not worried about this password being online.
# We only set one because the web interfaces HAVE to have one
ssh.connect("192.168.1.20", username="ubnt", password="rover4lyfe^", compress=True)

while True:
    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(get_general_info)

    output_json = json.loads(ssh_stdout.read())[0]

    successful_transmit_percent = output_json["ccq"]
    quality = output_json["airmax"]["quality"]
    capacity = output_json["airmax"]["capacity"]
    rx_rate = output_json["rx"]
    tx_rate = output_json["tx"]
    ground_tx_latency = output_json["tx_latency"]
    rover_tx_latency = output_json["remote"]["tx_latency"]

    print successful_transmit_percent, " | ", quality, " | ", capacity, " | ", rx_rate, " | ", tx_rate, " | ", ground_tx_latency, " | ", rover_tx_latency

    time.sleep(0.25)
# ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(set_wireless_frequency)
# ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(get_wireless_info)
#
# print ssh_stdout.read()

