## Multi-LoRa - Multi-radio and Multi-hop LoRa Communication Architecture for Large Scale IoT Deployment

<p align="justify">
Agriculture plays vital importance in developing an agricultural country, where rural modernization emerges as a solution to improve farming, field, and producers' issues. Multi-LoRa is a multi-radio and multi-hop LoRa communication architecture to increase the coverage and service for large-scale IoT deployment in rural areas. In this case, Multi-LoRa provides a reference for implementing communication networks based on the physical layer LoRa.
</p>

## Multi-LoRa architecture

<p align="justify">
The Multi-LoRa architecture provides a reference model for implementing multi-radio and multi-hop LoRa communication for large-scale IoT deployment in rural areas. Multi-LoRa architecture consists of four layers: Physical, MAC, Network, and Application, as shown the figure below. Specifically, the Physical layer considers the LoRa® radio provided by Semtech. For upper layers, there are some expected features. For instance, the MAC layer must ensure an efficient  transmission scheme over multiple radios. Furthermore, the Network layer is responsible for forwarding the routing data to route the data with lower signaling overhead and half-duplex communication. Finally, IoT applications can be built at the Application layer. In the following, we describe each layer of our architecture.
</p>

<p align="center">
    <img src="img/arch.png" height="600"/> 
</p>

## Multi-LoRa prototype

<p align="justify">
We developed a hardware prototype for physically implementing the Multi-LoRa architecture consists of hardware and software implementation, as shown in figure below. Each node shares the same hardware structure and main firmware. In terms of hardware implementation, each node is composed of an Espressif Esp32 microcontroller (dual-core 32-bit LX6 microprocessor, operating at 240 MHz and performing at up to 600 DMIPS, Ultra-Low-Power co-processor, 520 KB SRAM, 448 KB ROM), two LoRa radios SX1276, and energy source with a capacity of 2100 mAh. We implemented the MAC, routing, and application layer protocols on device firmware using the C programming language in software implementation. Specifically, we implemented the LBT communication model at the MAC layer, the Babel routing protocol (RFC8966) at the network layer, and Modbus at the application layer. 
</p>

<p align="center">
    <img src="img/sensor.png" height="300"/> 
</p>

## Main dependencies

### The minimum hardware requirement and software to run this experiment in bare metal (microcontroller) is shown in the image below.
Hardware:
- 4 Heltech Wireless Stick Lite (ESP32+SX1272) board
- 4 SX1276 module
- 4 USB cable - USB A / micro USB B
- Computer running Debian Linux
- ESP-IDF that essentially contains API (software libraries and source code) for ESP32 and scripts to operate the Toolchain

Software:
- ESP-IDF that essentially contains API (software libraries and source code) for ESP32 and scripts to operate the Toolchain

### The minimum hardware requirement and software to run this experiment in simulation mode is shown in the image below.
Hardware:
- Computer with 32GB RAM running Debian Linux

Software:
- Python 3.6 and libs in requeriments.txt
- gcc compiler an pthread lib


## First steps Hardware Prototype

<!-- Essa é a parte que o revisor quer ver-->
In order to use ESP-IDF with the Hardware prototype, you need to install some software packages on Debian Linux.

### Build and installation

<!-- Essa é a parte que o revisor quer ver-->
1 - To compile Pototype bare metal firmware using ESP-IDF you will need to get the following packages

```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```
2 - To build applications for the Pototype, you need the software libraries provided by Espressif in ESP-IDF repository.  To get ESP-IDF, navigate to your installation directory and clone the repository with git clone, following instructions.

Open Terminal, and run the following commands:
```
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
```
3 - Aside from the ESP-IDF, you also need to install the tools used by ESP-IDF, such as the compiler, debugger, Python packages, etc, for projects supporting ESP32.
```
cd ~/esp/esp-idf
./install.sh esp32
```

4 - The installed tools are not yet added to the PATH environment variable. To make the tools usable from the command line, some environment variables must be set. ESP-IDF provides another script which does that.

In the terminal where you are going to use ESP-IDF, run:
```
. $HOME/esp/esp-idf/export.sh
```
5 - Now you are ready to prepare your application for Prototype. You can start clone te project.
```
  git clone https://github.com/luciorp/multi-lora.git
  
  cd multi-lora
```
6 - Build the project by running:

```
idf.py build
```
This command will compile the application and all ESP-IDF components, then it will generate the bootloader, partition table, and application binaries.
```
$ idf.py build
Running cmake in directory /path/to/multi-lora/build
Executing "cmake -G Ninja --warn-uninitialized /path/to/multi-lora"...
Warn about uninitialized values.
-- Found Git: /usr/bin/git (found version "2.17.0")
-- Building empty aws_iot component due to configuration
-- Component names: ...
-- Component paths: ...

... (more lines of build system output)

[527/527] Generating multi-lora.bin
esptool.py v2.3.1

Project build complete. To flash, run this command:
../../../components/esptool_py/esptool/esptool.py -p (PORT) -b 921600 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x10000 build/multi-lora.bin  build 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin
or run 'idf.py -p PORT flash'
```

6 - Flash the binaries that you just built (bootloader.bin, partition-table.bin and multi-lora.bin) onto your prototype board by running:

```
idf.py -p PORT [-b BAUD] flash
```
Replace PORT with your ESP32 board’s serial port name.

### How to test

When flashing firmware in prototype, you will see the output log similar to the following:
```
esptool.py --chip esp32 -p /dev/ttyUSB0 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 2MB 0x8000 partition_table/partition-table.bin 0x1000 bootloader/bootloader.bin 0x10000 hello_world.bin
esptool.py v3.0-dev
Serial port /dev/ttyUSB0
Connecting........_
Chip is ESP32D0WDQ6 (revision 0)
Features: WiFi, BT, Dual Core, Coding Scheme None
Crystal is 40MHz
MAC: 24:0a:c4:15:b7:18
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
Configuring flash size...
Compressed 3072 bytes to 103...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (103 compressed) at 0x00008000 in 0.0 seconds (effective 5962.8 kbit/s)...
Hash of data verified.
Compressed 26096 bytes to 15408...
Writing at 0x00001000... (100 %)
Wrote 26096 bytes (15408 compressed) at 0x00001000 in 0.4 seconds (effective 546.7 kbit/s)...
Hash of data verified.
Compressed 147104 bytes to 77364...
Writing at 0x00010000... (20 %)
Writing at 0x00014000... (40 %)
Writing at 0x00018000... (60 %)
Writing at 0x0001c000... (80 %)
Writing at 0x00020000... (100 %)
Wrote 147104 bytes (77364 compressed) at 0x00010000 in 1.9 seconds (effective 615.5 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Done
```

To check if multi-lora firmware is indeed running, type idf.py -p PORT monitor (replace PORT with your serial port name).
The firmware's default configuration generates random traffic to all nodes in route table. 

<p align="center">
    <img src="img/learning.png" height="300"/> 
</p>



<!-- Essa é a parte que o revisor quer ver-->

## First steps Simulation

<!-- Essa é a parte que o revisor quer ver-->

### Build and installation

<!-- Essa é a parte que o revisor quer ver-->

### How to test

<!-- Essa é a parte que o revisor quer ver-->

## Main results

<p align="justify">
We evaluated the Multi-LoRa prototype considering two approaches: a small-scale based on a physical testbed and a large-scale using an simulation environment. The testbed results are investigated to calibrate the simulation experiments and analyze the performance of Multi-LoRa in real experiments. Moreover, the simulation experiments evaluated Multi-LoRa concerning delay and data delivery in a large-scale scenario. Furthermore, it is necessary to highlight that our simulation used the same source code prototyped in the testbed.
</p>

<p align="justify">
Figure below shows the delay for transmitting packets with different sizes considering three Multi-LoRa setups in the testbed environment. The delay results show that Average Packet Delay (APD) increases as soon as the packet size increases for transmitting data over Setup 1. Moreover, multi-radio architecture, i.e., Setup 2 and Setup 3 of Multi-LoRa, increases the APD lower than single-radio architecture for larger packet sizes. The weak performance of single-radio architecture is due to the half-duplex nature of the LoRa radio, where the queuing time on each hop is higher. For instance, packets with 256 bytes have APD 495% higher than the packet size of 32 bytes by transmitting the packet over Setup 1. We divide each data packet into two small packets to transmit over each radio on Setup 2 and Setup 3, reducing the transmission time. Finally, it is possible to conclude that Setup 3 provides lower APD regardless of the packet size compared to the other architectures. It considers packet transmission over multiple radios to mitigate the half-duplex nature of the LoRa radio and the BW value of 250 kHz helps to reduce APD.
</p>

<p align="center">
    <img src="img/testbed.png" height="300"/> 
</p>
    
<p align="justify">
Figure below shows APD for several IoT nodes considering three Multi-LoRa setups in the simulation environment. The APD results for large scale-scenario show that Setup 3 provides APD 61.5% and 26% lower than Setup 1 and Setup 2. This behavior happens because Setup 3 considers a multi-radio transmission with a better radio configuration (i.e., BW and SF values). For instance, the packet with a payload of 128 bytes transmitted with SF 7 and BW of 125 kHz has a ToA value of 332.03 ms, while SF 7 and BW of 250 kHz has a ToA average value of 166.02 ms. This result explains the reason for the APD performance of Multi-LoRa - Setup 3 is 50% better than Setup 2. Finally, Setup 2 provides APD results in 75% better than Setup 1 since Setup 2 considers two LoRa radios for transmitting data, reducing the effects of a half-duplex nature of the LoRa radio.
</p>

<p align="center">
    <img src="img/Simulation.png" height="300"/> 
</p>

## How to cite

<!--```
@article{prade2022Multi-LoRa,
    title={Multi-radio and Multi-hop LoRa Communication Architecture for Large Scale IoT Deployment},
    author={Lúcio Rene Prade and Jean Moraes and Eliel de Albuquerque and Denis Rosário and Cristiano Bonato Both},
    year={2022}
}
-->
