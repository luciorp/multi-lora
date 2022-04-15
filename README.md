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

<!-- precisa de uma descrição-->
What You Need
Hardware:
- An ESP32 board
- SX1276 module
- USB cable - USB A / micro USB B
- Computer running Linux

Software:
ESP-IDF that essentially contains API (software libraries and source code) for ESP32 and scripts to operate the Toolchain

## First steps

<!-- Essa é a parte que o revisor quer ver-->

## Build and installation

<!-- Essa é a parte que o revisor quer ver-->

## How to test

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
