# 🚗 Vehicule Signal STM32F466

[](https://www.google.com/search?q=https://github.com/YOUR_USERNAME/YOUR_REPO/actions)
[](LICENSE.md)
[](https://www.google.com/search?q=https://www.st.com/en/microcontrollers-microprocessors/stm32f466.html)

A firmware project for managing vehicle signaling systems using the **STM32F466** microcontroller. This includes control logic for indicators, hazard lights, and CAN bus communication for integration into a larger vehicle network.

-----

## 📋 Table of Contents

  * [Key Features](https://www.google.com/search?q=%23-key-features)
  * [Project Structure](https://www.google.com/search?q=%23-project-structure)
  * [Hardware & Software Requirements](https://www.google.com/search?q=%23-hardware--software-requirements)
  * [Getting Started](https://www.google.com/search?q=%23-getting-started)
  * [System Logic & Usage](https://www.google.com/search?q=%23-system-logic--usage)
  * [CAN Frame Specification](https://www.google.com/search?q=%23-can-frame-specification)
  * [Contributing](https://www.google.com/search?q=%23-contributing)
  * [License](https://www.google.com/search?q=%23-license)

-----

## ✨ Key Features

  * **Turn Indicators**: Independent control for left and right signals.
  * **Hazard Warning**: Synchronized flashing of all indicator lights.
  * **CAN Bus Integration**: Broadcasts signaling status to other vehicle ECUs.
  * **Real-time Control**: Built using STM32 HAL libraries for reliable and efficient performance.
  * **Scalable**: Easily extendable to include other lighting systems (e.g., brake lights, daytime running lights).

-----

## 📦 Project Structure

The project follows a standard STM32CubeIDE directory structure to keep the code organized and maintainable.

```
vehicule-signal-stm32/
├── Core/
│   ├── Inc/                    # Core header files (main.h, stm32f4xx_it.h)
│   └── Src/                    # Core source files (main.c, stm32f4xx_it.c)
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/   # ST's Hardware Abstraction Layer drivers
│   └── CMSIS/                  # Cortex Microcontroller Software Interface Standard
├── Middlewares/
│   └── Third_Party/            # Optional: Middleware libraries (e.g., FreeRTOS)
├── CAN_Logic/
│   ├── Inc/                    # CAN message encoding/decoding headers
│   └── Src/                    # CAN message encoding/decoding sources
├── DOC/
│   └──                      # Project documentation, datasheets, schematics
├── .cproject                   # Eclipse C/C++ project file
├── .project                    # Eclipse project file
└── vehicule-signal-stm32.ioc   # STM32CubeMX configuration file
```

-----

## 🛠️ Hardware & Software Requirements

### Hardware

  * **Microcontroller**: STM32F466 Nucleo Board or custom PCB.
  * **CAN Transceiver**: A 5V CAN transceiver module (e.g., TJA1050, MCP2551) to interface with the CAN bus.
  * **Input**: Push buttons for Left Indicator, Right Indicator, and Hazard lights.
  * **Output**: LEDs to simulate vehicle indicator lights (Front-Left, Rear-Left, Front-Right, Rear-Right).
  * **Programmer/Debugger**: ST-LINK/V2 (usually integrated into Nucleo boards).
  * **Power Supply**: 5V or 3.3V power source.

### Software

  * **IDE**: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (Version 1.10.0 or later recommended).
  * **Configuration Tool**: STM32CubeMX (integrated within the IDE).
  * **Compiler**: GCC for ARM.
  * **Serial Terminal**: A terminal emulator (e.g., PuTTY, Tera Term) for optional debug messages.

-----

## 🚀 Getting Started

Follow these steps to set up, build, and flash the firmware.

### Prerequisites

Ensure you have all the software listed under the [Software Requirements](https://www.google.com/search?q=%23-hardware--software-requirements) section installed on your development machine.

### Clone & Setup

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/YOUR_USERNAME/YOUR_REPO.git
    cd vehicule-signal-stm32
    ```
2.  **Open in STM32CubeIDE:**
      * Launch STM32CubeIDE.
      * Go to `File > Import...`.
      * Select `General > Existing Projects into Workspace` and click `Next`.
      * For `Select root directory`, browse to the cloned repository folder.
      * Check the box for the discovered project and click `Finish`.

### Build & Flash

1.  **Build the Project**:
      * Right-click on the project in the *Project Explorer*.
      * Select `Build Project` or press `Ctrl+B`.
2.  **Flash the Firmware**:
      * Connect your STM32 board to your computer via the ST-LINK USB port.
      * Right-click on the project and select `Run As > STM32 Application`.
      * The IDE will automatically build and flash the firmware to the board.

-----

## ⚙️ System Logic & Usage

The firmware operates based on button inputs to control the signaling state.

  * **Left Indicator**: Pressing the "Left" button starts the left-side LEDs blinking at a 1Hz frequency. Pressing it again deactivates them.
  * **Right Indicator**: Pressing the "Right" button starts the right-side LEDs blinking at a 1Hz frequency. Pressing it again deactivates them.
  * **Hazard Lights**: Pressing the "Hazard" button activates all four indicator LEDs simultaneously at 1Hz. Pressing it again deactivates them. Hazard mode overrides any active turn signal.
  * **CAN Broadcast**: The current status of the signals is broadcast over the CAN bus every 200ms.

-----

## 📡 CAN Frame Specification

The firmware transmits a single CAN frame to report the vehicle's signaling status. This allows other systems (like a dashboard or body control module) to be aware of the driver's intentions.

**Transmitted Frame: `SignalingStatus_T`**

| CAN ID | DLC | Byte 0 | Bytes 1-7 | Description |
| :--- | :-: | :--- | :--- | :--- |
| **`0x701`** | 1 | `SignalState` | `(Unused)` | Reports the current state of the vehicle's signals. |

**`SignalState` (Byte 0) Values:**

| Value (Hex) | Value (Dec) | Status |
| :--- | :-: | :--- |
| `0x00` | 0 | **Off**: All signals are inactive. |
| `0x01` | 1 | **Left Indicator On**: Left signal is active. |
| `0x02` | 2 | **Right Indicator On**: Right signal is active. |
| `0x03` | 3 | **Hazard Lights On**: Hazard warning is active. |

-----

## 🤝 Contributing

Contributions are welcome\! If you'd like to improve the project, please follow these steps:

1.  Fork the repository.
2.  Create a new branch (`git checkout -b feature/AmazingFeature`).
3.  Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4.  Push to the branch (`git push origin feature/AmazingFeature`).
5.  Open a Pull Request.

-----
📬 Contact: For collaboration or technical queries, reach out to https://github.com/mraihiimed/.

📜 License: See LICENSE file. Default usage under STMicroelectronics terms or AS-IS if none present.

## 📄 License

This project is distributed under the MIT License. See the `LICENSE.md` file for more details.
