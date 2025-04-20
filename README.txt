
# Final Project: STM32 Performance Prediction and Circuit Design

This project combines STM32CubeIDE software for microcontroller configuration with a Fritzing circuit diagram for hardware visualization. The goal is to showcase how the STM32F446ZETx microcontroller can be used for various peripherals such as ADC, PWM, and GPIO operations. This project also includes a visual circuit design using Fritzing, which helps to understand the wiring and connections in the hardware setup.

## Features

- **Microcontroller**: STM32F446ZETx (part of the STM32F4 series)
- **Key Peripherals**:
  - **ADC1**: Used for analog to digital conversion with channels IN3 (PA3) and IN10 (PC0)
  - **TIM3**: Used for PWM Generation (Channel 2 on PA7)
  - **GPIO Outputs**: Multiple GPIO pins configured for output (PA5, PA6, PF12, PF13, PF14, PF15, PE9, PE11, PE13, PD15, PG14)
  - **GPIO Input**: PD14 pin configured for input
- **Fritzing Circuit Design**: A visual representation of the circuit connections used in this project, helping users understand the hardware setup.

## Hardware Components

- **Board**: STM32F446ZET6
- **Sensor**: LM35DZ (Temperature Sensor)
- **Motor**: Servo Motor
- **RGB LED**: Common Cathode
- **Software IDE**: STM32CubeIDE
- **Programming Language**: C/C++

### Application Purpose:
The purpose of this application is to monitor and control the temperature using various components. The temperature range is set using a potentiometer, and the selected ranges are displayed on an LCD screen. The user confirms the selection using a button. Based on the measured temperature:
- If the temperature is below the lower range limit, the RGB LED lights up blue, the red LED remains off, the buzzer is silent, and the servo motor turns to the west.
- If the temperature is within the selected range, the RGB LED lights up green, the red LED turns on, the buzzer remains silent, and the servo motor turns to the north.
- If the temperature exceeds the upper range limit, the RGB LED lights up red, the red LED flashes continuously, the buzzer sounds, and the servo motor turns to the east.

## Requirements

- **Toolchain**: STM32CubeIDE
- **Firmware Package**: STM32Cube FW_F4 V1.28.1
- **Microcontroller**: STM32F446ZETx (144-pin LQFP package)
- **Peripherals**:
  - ADC (Analog to Digital Converter)
  - PWM (Pulse Width Modulation)
  - GPIO (General Purpose Input Output)
- **Fritzing Software**: To view the circuit design, install Fritzing from [here](https://fritzing.org/download/).

## Configuration Details

1. **Pinout Configuration**: The STM32F446ZETx microcontroller's pins are configured to interact with various peripherals. The most notable configurations are:
   - **ADC1**: Channels ADC1_IN3 (PA3) and ADC1_IN10 (PC0)
   - **TIM3**: Channel 2 for PWM generation on PA7
   - **GPIO Outputs**: Multiple pins (PA5, PA6, PF12, PF13, PF14, PF15, PE9, PE11, PE13, PD15, PG14) are configured for output, allowing flexibility in digital control.
   - **GPIO Input**: PD14 is set as an input pin.

2. **Clock Configuration**: The project is designed with appropriate clock settings, including HSI, HSE, PLL configurations, and prescalers to ensure stable peripheral operations.

3. **Power Consumption**: Power consumption is carefully calculated to ensure optimal performance while maintaining battery life for portable applications. This configuration includes power regulator settings and the use of low-power modes.

4. **Software Configuration**:
   - **Code Generation Settings**: Ensures that only necessary library files are included, optimizing the size of the generated firmware.
   - **Toolchain Settings**: Configured to ensure efficient use of resources through compiler optimizations.

## Pin Configuration Summary

- **PC0 (Pin 26)**: ADC1_IN10, GPIO Analog
- **PA3 (Pin 37)**: ADC1_IN3
- **PA5, PA6 (Pins 41, 42)**: GPIO Outputs
- **PA7 (Pin 43)**: TIM3 Channel 2 (PWM Output)
- **PF12, PF13, PF14, PF15 (Pins 50-55)**: GPIO Outputs
- **PE9, PE11, PE13 (Pins 60, 64, 66)**: GPIO Outputs
- **PD14 (Pin 85)**: GPIO Input
- **PD15 (Pin 86)**: GPIO Output
- **PG14 (Pin 129)**: GPIO Output

## Fritzing Circuit Diagram

You can view the visual representation of the circuit by opening the **`final_stm.fzz`** file in the Fritzing software. The Fritzing diagram helps in understanding the connections and provides a clear guide for assembling the hardware.



## How to Use

1. **Clone the Project**: Download or clone the repository to your local machine.
2. **Open in STM32CubeIDE**: Open the project in STM32CubeIDE (ensure the correct STM32Cube firmware package is installed).
3. **Generate Code**: Use STM32CubeIDE to generate the initialization code for peripherals.
4. **Build and Flash**: Build the project and flash it onto the STM32F446ZETx microcontroller.
5. **Fritzing Circuit Design**: Open the Fritzing software and load the **`final_stm.fzz`** file to view the circuit diagram and connections.

## License

This project is open source and can be used for educational and research purposes. Feel free to adapt and extend it for your own microcontroller-based projects.
