# Scanner 3D - HC-SR04

Sistema de escaneamento 3D usando sensor ultrassônico HC-SR04, Arduino e interface Python com PyQt6 + Vispy.

## 🔌 Hardware

**Conexões HC-SR04:**

```
HC-SR04 -> Arduino
├─ VCC  -> 5V
├─ GND  -> GND
├─ TRIG -> Pin 11
└─ ECHO -> Pin 10
```

## 🚀 Instalação

```bash
cd "/Users/pampas/Desktop/Projetos EB"
source venv/bin/activate
pip install -r requirements.txt
```

## ▶️ Executar

**Arduino:**

```
1. Upload: Arduino/scanner_3d_sensor.ino
2. Configurar baudrate: 115200
```

**Python:**

```bash
python Python/scanner_3d.py
```

## 📊 Especificações

- **Pontos por camada:** 7 (51.43° cada)
- **Altura entre camadas:** 8.0 mm
- **Taxa de amostragem:** 10 Hz
- **Raio do braço:** Configurável (padrão: 200mm)

## 💾 Exportação

- **CSV:** Todos os pontos (X, Y, Z em mm)
- **STL:** Mesh 3D (requer 2+ camadas completas)

---

**Criado em:** 18 de dezembro de 2025
