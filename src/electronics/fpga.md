# FPGA #
 
 ## 8.FPGA: What is it? ##
**Field Programmable Gate Array** - Programmable Gate System These are programmable (or rather reconfigurable) integrated circuits
many times and more and more dynamically! 
**They allow**
Either to emulate a circuit to validate it before silicon etching
Either to implement a complete system, System on Chip (SoC)
By using dynamic reconfiguration, this system can become adaptive as needed (acceleration of selected features).

### 8.1. Introduction:###
FPGAs consist of a matrix of programmable logic blocks surrounded by programmable output input blocks. The whole is connected by a programmable interconnection network.
FPGAs are distinct from other programmable circuit families while offering the highest level of logical integration.

There are 4 main categories available commercially:

Symmetrical table.
In column.
Seas of doors.
Hierarchical PLDs.

                     The different classes of FPGAs.img
  ### 8.2.FPGA structure  ###                  
  
 The specifications of an FPGA often include the number of configurable logic blocks, the number of logic blocks of frozen functions such as multipliers and the size of the memory resources such as the embedded RAM block. An FPGA integrated circuit is composed of several other elements, but these are typically the most important when it comes to choosing and comparing FPGAs for a specific application.

Configurable logic blocks (CLBs) are the basic logical unit of an FPGA. Sometimes called "slices" or "logical cells", logical blocks consist of two parts: flip-flops and correspondence tables (LUTs). The way flip-flops and LUT lookup tables are assembled differs depending on the types of FPGAs, so it's important to understand their features.    
                                        fpga_logic_blocks_interconnects.img 
### 8.4. FPGAs: CLBs - Logic Blocks (Re) Configurable - but not only! ###     
                                              capture__n
### 8.3.The Possibilities ###
   With an FPGA you are able to create the actual circuit, so it is up to you to decide what pins the serial port connects to. That also means you can create as many serial ports as you want. The only limitations you really have are the number of physical I/O pins and the size of the FPGA.

Just like microcontrollers that have a set amount of memory for your program, FPGAs can only emulate a circuit so large.

One of the very interesting things about FPGAs is that while you are designing the hardware, you can design the hardware to be a processor that you then can write software for! In fact, companies that design digital circuits, like Intel or nVidia, often use FPGAs to prototype their chips before creating them.      
3. Résumé
 FPGA continues to dominate as the evolution of high-level tools allows engineers and scientists to enjoy the benefits of reprogrammable circuits, regardless of their level of experience. For more information on using FPGAs in different industries and applications    
 
## 1 INTRODUCTION ## 
The protocol that we will use is called Serial Peripheral Interface (SPI). It is a synchronous full-duplex serial interface and is commonly used to communicate with on-board peripherals such as EEPROM, FLASH memory, A/D converters, temperature sensors, or in our case a Field Programmable Gate Array (FPGA).
We assume a working knowledge of the VHDL hardware description language. 

## 2 HARDWARE ## 
SPI is a protocol, in which one device (the master) controls one or more other devices (the slaves). For the master we use an open-source microcontroller prototyping platform, such as the Arduino 101 or a modified Arduino UNO R3. In this document we use Arduino to refer to either platform.
The slave can be a low-cost FPGA prototyping platforms, such as the Xilinx Spartan-6 Avnet LX9 or the Altera Cyclone-IV Terasic DE10-Lite . The repository  includes project files and pin assignments for both these boards. The code is written in VHDL and should work equally well on more powerful boards.

### 2.1.Voltage levels ### 
It is very important that the I/O voltage levels of the devices match. Both FPGA boards support 3.3V levels, and are a good match for the Arduino 101. However, the Arduino UNO uses the traditional 5 Volt TTL levels. Instead of using a level shifter, such as the 74LVC245, we opt for converting the Arduino to 3.3V according to Adafruit’s instructions. Running a 16 MHz clock at 3.3V is out of spec. Is said to work, but should really program the fuses to get the frequency down to abt. 13 MHz .

### 2.2.Signals ### 
The SPI interface is a 4 wire interface. The bus consists of 3 signals plus *a slave select* signal for each device.

*SCLK*: clock signal sent from the master to all slaves.

*MOSI*: serial data from the master to the slaves (Master Out-Slave In).

*MISO*: serial data from a slave to the master (Master In-Slave Out).

*SS*:  slave select signal for each slave.

Once the Arduino runs at 3.3V, connecting the two devices becomes trivial.

### 2.2.1. Pin outs ### 

![z](img/electronics/fpga/image1.png)

### 2.2.2. The physical connections ### 

![z](img/electronics/fpga/image2.png)

      
## 3. BYTE – PROTOCOL ## 

With the two devices physically connected, we need a protocol to transfer data. We chose the Serial Peripheral Interface (SPI), a lightweight protocol to connect one master to one or more slaves.

### 3.1 Master/slave ### 
The SPI bus is controlled by a master device (typically a microcontroller) that orchestrates the bus access. The master generates the control signals and regulates the data flow. The illustration below shows a master with three slaves. The master uses the Slave Select (SS) signal to select the slave.
 
![z](img/electronics/fpga/image3.png)

### 3.2 Parameters ###
SPI is also a protocol with many degrees of freedom. It is important that the master and slave agree on the voltage levels and maximum clock frequency. The SPI clock polarity (CPOL) and clock phase (CPHA) introduce four more degrees of freedom as shown in the table below.
SPI parameters

![z](img/electronics/fpga/image4.png)

For this article we assume *mode 3*, where the clock is high when idle; data is driving following the falling edge of the clock and latched on the rising edge.

### 3.3. Operation ###
The protocol is easiest explained with shift registers as shown in the illustration below. The master generates the SPI Clock (*SCLK*) to initiate the information exchange. Data is shifted on one edge of this clock and is sampled on the opposite edge when the data is stable.

 ![z](img/electronics/fpga/image5.png)

In mode 3, at the falling edge of *SCLK*, both devices drive their most significant bit (*b7*) on their outgoing data line. On the rising edge, both devices clock in this bit into the least significant bit position (*b0*). After eight *SCLK* cycles, the master and slave have exchanged their values and each device processes the data received (e.g. writing it to memory). In case there is more data to be exchanged, the registers are loaded with new data and the process repeats itself. Once all data is transmitted, the master stops the *SCLK* clock.

### 3.4. Slave select ###
For a more complete picture, we need to include the effect of the *slave select* (*SS**) signal that is used to address the slave devices.
 
 ![z](img/electronics/fpga/image6.png)

Slaves may only drive their output (*MISO*) line when *SS** is active, otherwise they should tri-stated the output. The protocol can be broken down into the following steps:

1.	 The master initiates the communication by activating SS*

*  The slave responds by starting to drive its MISO output.
+	Meanwhile the master drives its MOSI output.

2.	 The master makes SCLK low.

+ On this falling edge, the master and slave drive their most significant bit position (b7) on respectively their MOSI and MISO outputs.

3.	 The master makes SCLK high.

+ On this rising edge, the master and slave clock the input from their respectively MISO and MOSI inputs into the least significant bit position (b0).

4.	 Go back to step 2. Until the least significant bit position (b0) has been sent.

5.	 When all bits are transmitted, the master deactivates SS*.

## 4. BYTES EXCHANGE WITH ARDUINO AS MASTER ##
The Arduino is blessed with a support library for the serial peripheral interface. This greatly aids the implementation. For the slave we used an Altera or Xilinx based FPGA implementation . Refer to the first part of this article for details about the physical connection. 

## 5. BYTE EXCHANGE WITH A FPGA AS SLAVE ##
Implementing the SPI Slave on an FPGA is like old school digital electronics. My key takeaway is to think hardware, not programming. Implementing the [SPI protocol](https://coertvonk.com/hw/logic/connecting-fpga-and-arduino-using-spi-13067/3) on a FPGA is fairly straightforward for as long as we use a directly clocked sequential circuit while preventing clock domain crossings.

### 5.1. Sequential circuit ### 
In real life, two signals going to a single gate will not arrive there at the same time due to wire delays. This causes the output to momentarily have an incorrect value. The problem compounds as the signal travels through more gates and wires.

In Building Math Hardware we created elementary math operations using combinatorial circuits. That was OK, because we didn’t care about such output *glitches* caused by the input signals propagating to the outputs. From a demonstrator’s point of view it even made it more interesting. Talking to a real device, such as a SPI master is different, because it requires the outputs to be stable at certain times.
 
 ![z](img/electronics/fpga/image7.png)

The solution is to introduce a clock signal, and store the signals in a flip-flop (registers) at the rising edge of that clock signal. We then only need to ensure that the longest delay from one flip-flop to the next is less that the clock period. This greatly simplifies the design process, at the cost of introducing some delay.

### 5.2. Clock domain ###
Field programmable gate arrays thrive on synchronous designs, but they don’t do well with clock signals that are asynchronous with its system clock.
 
We also need to avoid transferring data from a [flip-flop](https://coertvonk.com/family/school/inquiries/computer-math-inquiry-4245) driven by one clock to a flip-flop driven by another clock. This is called a *clock domain crossing* and might manifest itself in *metastability*, data loss or incoherence .We prevent clock domain crossings, by synchronizing the input signals to the FPGA clock using a traditional two-stage shift register as illustrated above.

+ The first flip-flop creates a synchronous version of the inputs by clocking it with the system clock. The input signal could change within the flip-flop’s *setup and hold times* and may take longer than a system clock cycle to settle to a stable value (metastability). That’s why it is ran through a second flip-flop.

+ The second flip-flop, makes it is very unlikely that this metastability propagates to the output.

+ Adding a third flip-flop gives us access to the previous value. Using the current and previous values, we can generate rise and fall signals as sown below.

  ![z](img/electronics/fpga/image8.png)

### 5.3. Operation ###
The main data object is an 8-bit register called DATA.

On a falling SCLK edge, the most significant bit from data is clocked into a register from where it is transmitted over its MISO output.

On a rising SCLK edge, the MOSI input is shifted into the least significant bit of data.

Once all eight bits are received, the byte is available as rx. This received byte rx should be read when rx Valid is active during a rising edge of the sysClk.

## 6. MESSAGE EXCHANGE PROTOCOL ##
The time has come to implement a status and register interface on top of the raw byte exchange. We define a few commands to retrieve the device status and access its 32-bit registers.

### 6.1. Commands ###
The first byte is defined as the command byte. The interpretation of the remaining bytes (if any) depends on this command. After the command is completed, a new command can be sent. The following commands will be supported:

+	**Read status (0x00)**: Reads the status byte.

First the master sends 0x00, and ignores the value returned;

the master then sends one dummy byte, to get the 8-bit status value in return.

+ **Read register (0x80 to 0x8F)**: Reads the value stored in one of the sixteen registers. 

The least significant four bits of the command indicate the register to read from.
First the master sends this command, and ignores the value returned,then the master sends 4 dummy bytes to get the 32-bit register value. The first byte received is the most significant, the fourth is the least significant (network byte order).

+ **Write register (0xC0 to 0xCF)**: Writes a value to one of the sixteen registers. The least significant four bits indicate the register to write to.

  * First the master sends the command, and ignores the value returned;
  * Then the master sends 4 bytes with the value to write. The most significant byte is sent first, the least significant last.

### 6.2. Registers ###
The FPGA will implement two register types. The first 4 registers (0-3) are *read/write* and can be used to send information to the FPGA. The next 12 registers (4-15) are *read-only* to receive information from the FPGA. In the greater scheme, the *read/write* registers will be used to send math operands to the FPGA, and the read-only registers will be used to read the results from the FPGA.

### 6.3. Room for improvement ###
The protocol leaves some room for improvement. It can be made more efficient by implementing continuous commands. Here the master issues a *read* or *write register start* command, but keeps sending sets of 4-bytes until it has enough. Along similar lines one could implement commands to access attached memory.

## 7. MESSAGES EXCHANGE WITH ARDUINO AS MASTER ##

Again, we’ll use the support library for the serial peripheral interface. The code shown below was tested on a 3.3 Volt Arduino UNO R3 connected to FPGA implementation. 



                                        
                                        
                                        
                                        
                                        
