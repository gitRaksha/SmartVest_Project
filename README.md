
![Made with STM32](https://img.shields.io/badge/Made%20with-STM32-blue?logo=stmicroelectronics&style=flat-square)  ![ESP32](https://img.shields.io/badge/Powered%20by-ESP32-orange?logo=espressif&style=flat-square)  ![LoRa](https://img.shields.io/badge/LoRa-SX1278-purple?logo=semtech&style=flat-square)  ![Made with ❤️ in Tunisia](https://img.shields.io/badge/Made%20with%20❤️-Tunisia-red?style=flat-square) 



# 🦺 Smart Safety Vest – IoT Project  

A **Smart Safety Vest** designed for workers in hazardous environments.  
It detects **toxic gas (CO)** and **falls**, then transmits alerts via **LoRa (SX1278)** from an **STM32F407G** to an **ESP32**, which hosts a **web dashboard** for monitoring.  

This project was developed as an **End-of-Year Engineering Project (PFA)**.  

---

## ✨ Features
- 📡 **LoRa Communication**: Long-range wireless transmission of alerts.  
- 🧭 **Fall Detection**: Using MPU6050 accelerometer.  
- 🌫 **Gas Detection**: CO levels via MQ-7 sensor.  
- 🌍 **Web Dashboard**: ESP32 receives LoRa data and displays it on a local web page.  
- 🔋 **Portable Design**: PCB + 3D printed case for deployment.  

---

## 🛠 Hardware Used
🔹 **STM32F407G** microcontroller (main vest controller)  
🔹 **ESP32** (LoRa receiver + Wi-Fi access point / web interface)  
🔹 **LoRa SX1278** transceiver modules (433 MHz)  
🔹 **MQ-7** gas sensor (CO detection)  
🔹 **MPU6050** accelerometer/gyroscope (fall detection)  
🔹 **Buzzer + LED indicators** (local alerts)  
🔹 **Li-Ion Battery 3.7V** + **TP4056 charger module**  
🔹 **Custom PCB** (Proteus/Altium design included)  
🔹 **3D Printed Enclosure** (OpenSCAD design provided)  

---

## 💻 Software & Tools
- 🖥️ **STM32CubeIDE** or **Keil uVision** (STM32 firmware)  
- 🖥️ **Arduino IDE** (ESP32 firmware)  
- 🖥️ **Proteus** / **Altium Designer** (PCB design)  
- 🖥️ **OpenSCAD** (3D case design)  
- 🖥️ **Node-RED (optional)** (IoT dashboards)  

---

## 📂 Project Structure

```text
.
├── 3D design OpenScad # Enclosure design (box + lid)
├── CODE # Source codes
│ ├── esp32_final # ESP32 receiver + web server
│ └── stm32_final_code # STM32 sensor reading + LoRa sender
├── Images # Screenshots, wiring photos, etc.
├── PCB # PCB design files (Altium + Proteus)
├── Rapport_PFA.pdf # Final written report
└── Vidéo.mp4 # Demo video of the project

 ```

---

## 🚀 Deployment  

### 🔹 STM32 (Sender – on the vest)  
1. Open `stm32_final_code` in **STM32CubeIDE**.  
2. Flash the firmware to your STM32F407G.  
3. Connect MQ-7 + MPU6050 + LoRa SX1278 as per schematic.  

### 🔹 ESP32 (Receiver – Web Dashboard)  
1. Open `esp32_final` in **Arduino IDE**.  
2. Install required libraries:  
   - `LoRa.h`  
   - `WiFi.h`  
   - `WebServer.h`  
3. Flash to ESP32.  
4. Connect ESP32 to LoRa SX1278.  
5. Open a browser and navigate to the ESP32 IP to view alerts.  

---

## 📸 Images & Demo
Some project visuals are stored in the [Images](./Images) folder.  
A demo video is available: [Vidéo.mp4](./Vidéo.mp4).  

---

## 🔮 Future Improvements

🌍 Add GPS for geolocation

📡 Add GSM/4G for connectivity without LoRa

🔋 Optimize power management

☁️ Cloud-based dashboard (IoT integration)

---

## 📜 License
This project is licensed under the **MIT License** – see the [LICENSE](./LICENSE) file for details.  

---

## 👥 Authors
- **Marwen Mekni** 
- **Bensaid Mohamed**
- **Khemiri Haythem** 

---

⭐ If you like this project, feel free to fork, star, or reuse it for your own IoT projects!
