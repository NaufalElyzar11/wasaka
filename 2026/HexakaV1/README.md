# 🦾 HexakaV1 — Robot Hexapod dengan Dynamixel AX-12A

Proyek kendali robot hexapod berbasis Python menggunakan servo **Dynamixel AX-12A** dan antarmuka **U2D2**. Robot ini memiliki 6 kaki, masing-masing terdiri dari 3 sendi (Coxa, Femur, Tibia), sehingga total 18 servo.

---

## 📐 Spesifikasi Mekanik

| Segmen | Panjang |
|--------|---------|
| **Coxa** (L1) | 3.4 cm |
| **Femur** (L2) | 4.0 cm |
| **Tibia** (L3) | 5.45 cm |

### Pemetaan ID Servo

| Kaki | Coxa | Femur | Tibia |
|------|------|-------|-------|
| L1 — Depan Kiri | 1 | 2 | 3 |
| L2 — Tengah Kiri | 4 | 5 | 6 |
| L3 — Belakang Kiri | 7 | 8 | 9 |
| L4 — Depan Kanan | 10 | 11 | 12 |
| L5 — Tengah Kanan | 13 | 14 | 15 |
| L6 — Belakang Kanan | 16 | 17 | 18 |

### Batasan Servo

| Sendi | Min | Max |
|-------|-----|-----|
| Femur | 191 | 854 |
| Tibia | 82 | 929 |

---

## 🗂️ Struktur File

```
HexakaV1/
├── Coba.py       # Simulasi IK offline (tanpa hardware)
├── tes.py        # Pengujian posisi berdiri 1 kaki (Kaki 1)
├── stand.py      # Posisi standby semua 6 kaki serentak
├── gait.py       # Simulasi siklus langkah 1 kaki (5 siklus)
├── tripod.py     # Gait tripod — pengiriman serial (sequential)
└── jalan.py      # Gait tripod — pengiriman sinkron (GroupSyncWrite)
```

---

## ⚙️ Konfigurasi Hardware

| Parameter | Nilai |
|-----------|-------|
| Protocol | Dynamixel Protocol 1.0 |
| Baudrate | 1,000,000 bps |
| Port | `COM21` *(sesuaikan)* |
| Center Value | 512 |
| Resolusi | 1023 / 300° |

> **⚠️ Penting:** Ubah nilai `DEVICENAME = 'COM21'` di setiap script sesuai port COM aktual U2D2 Anda.

---

## 🧮 Algoritma Inverse Kinematics

Robot menggunakan **spider stance** (lutut menekuk ke atas). Perhitungan IK dilakukan per kaki dengan input koordinat `(x, y, z)` dalam satuan **cm**.

```
γ  = atan2(y, x)                          → Sudut Coxa
L_eff = √(x²+y²) - L1                     → Jarak efektif setelah coxa
D  = √(L_eff² + z²)                       → Jarak diagonal femur–tibia
θ_femur = atan2(z, L_eff) - acos(...)    → Spider stance (lutut atas)
θ_tibia = π - acos(...)                   → Sudut tibia
```

**Polaritas Dynamixel:**
- **Coxa:** `512 - (deg × 3.41)`
- **Femur:** `512 + (deg × 3.41)`
- **Tibia:** `512 - (deg × 3.41)`

---

## 🚀 Cara Penggunaan

### Prasyarat

```bash
pip install dynamixel-sdk
```

### 1. Simulasi IK (tanpa robot)

```bash
python Coba.py
```

Menghitung dan menampilkan nilai Dynamixel untuk target posisi ujung kaki tertentu. **Tidak memerlukan hardware.**

### 2. Uji Posisi 1 Kaki

```bash
python tes.py
```

Menggerakkan **Kaki 1** (Depan Kiri) ke posisi berdiri `X=8cm, Z=7cm`. Cocok untuk verifikasi orientasi servo sebelum menjalankan semua kaki.

### 3. Posisi Standby (Semua Kaki)

```bash
python stand.py
```

Mengirim perintah ke seluruh 18 servo agar robot berdiri di posisi standby `X=7cm, Z=5cm`. Direkomendasikan sebagai **langkah pertama** saat menghidupkan robot.

### 4. Simulasi Langkah 1 Kaki

```bash
python gait.py
```

Menjalankan siklus langkah pada Kaki 1 sebanyak **5 siklus** menggunakan waypoints: angkat kaki → ayun ke depan → injak tanah → dorong.

### 5. Berjalan — Tripod Gait (Serial)

```bash
python tripod.py
```

Menjalankan gait tripod pada semua 6 kaki secara **berurutan (sequential)**. Tekan `Ctrl+C` untuk berhenti.

- **Grup A** (fase 0): L1 Depan Kiri, L3 Belakang Kiri, L5 Tengah Kanan
- **Grup B** (fase π): L2 Tengah Kiri, L4 Depan Kanan, L6 Belakang Kanan

### 6. Berjalan — Tripod Gait (Sinkron / Direkomendasikan)

```bash
python jalan.py
```

Sama seperti `tripod.py`, namun menggunakan **`GroupSyncWrite`** untuk mengirim data ke semua servo secara **bersamaan dalam satu paket**. Menghasilkan gerakan yang lebih mulus dan responsif.

---

## 🔧 Parameter Gait (dapat disesuaikan)

| Parameter | Default | Keterangan |
|-----------|---------|------------|
| `X_REST` | 7.0 cm | Jarak kaki dari bodi saat istirahat |
| `Z_GROUND` | 5.0 cm | Tinggi bodi dari tanah |
| `LIFT_HEIGHT` | 2.5 cm | Ketinggian angkat kaki |
| `STRIDE_LENGTH` | 6.0 cm | Panjang langkah (maju-mundur) |
| `SPEED` | 0.08–0.3 | Kecepatan fase sinyal (semakin kecil = semakin halus) |
| `DELAY` | 0.01 s | Jeda antar iterasi loop |

---

## 📋 Urutan Pengujian yang Direkomendasikan

```
1. Coba.py    → Verifikasi matematis IK
2. tes.py     → Uji 1 kaki (robot digantung)
3. stand.py   → Uji semua kaki berdiri (robot digantung)
4. gait.py    → Uji siklus langkah 1 kaki
5. tripod.py  → Uji berjalan (robot digantung/di permukaan)
6. jalan.py   → Operasi normal berjalan
```

---

## 📄 Referensi

- [Dynamixel SDK Documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [AX-12A E-Manual](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
- Dokumen Perancangan: `Perancangan Gerakan Hexapod berdasarkan Spesifikasi.pdf`
