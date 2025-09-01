# คู่มือการใช้งานคำสั่ง GNSS Testing

## ภาพรวม
โค้ดนี้เพิ่มฟีเจอร์การทดสอบ GNSS (GPS/GLONASS/BeiDou/Galileo/QZSS) ให้กับ SIMCom A76XX cellular module โดยใช้คำสั่งคอนโซลผ่าน USB Serial JTAG

## คำสั่งที่เพิ่มใหม่

### `gnss_init`
**คำอธิบาย:** เริ่มต้นระบบ GNSS
**การใช้งาน:** `gnss_init`
**คำสั่ง AT ที่ใช้:**
- `AT+CGNSSPWR=1,1,1` - เปิดพลังงาน GNSS พร้อม fast hot start
- `AT+CGNSSMODE=3` - ตั้งโหมดเป็น GPS L1 + QZSS
- `AT+CGNSSNMEA=1,1,1,1,1,1,0,0` - เปิด NMEA sentences (GGA,GLL,GSA,GSV,RMC,VTG)
- `AT+CGPSNMEARATE=1` - ตั้งอัตรา NMEA เป็น 1Hz

### `gnss_start`
**คำอธิบาย:** เริ่มการระบุตำแหน่ง GNSS
**การใช้งาน:** `gnss_start`
**คำสั่ง AT ที่ใช้:**
- `AT+CGNSSPORTSWITCH=0,1` - กำหนดพอร์ตเอาต์พุต (Parsed → USB AT, Raw NMEA → UART)
- `AT+CGNSSTST=1` - เริ่มสตรีม NMEA ไปยัง UART

### `gnss_stop`
**คำอธิบาย:** หยุดการทำงานของ GNSS
**การใช้งาน:** `gnss_stop`
**คำสั่ง AT ที่ใช้:**
- `AT+CGNSSTST=0` - หยุดสตรีม NMEA
- `AT+CGNSSPWR=0` - ปิดพลังงาน GNSS

### `gnss_location`
**คำอธิบาย:** อ่านตำแหน่งปัจจุบัน
**การใช้งาน:** `gnss_location`
**คำสั่ง AT ที่ใช้:**
- `AT+CGNSSINFO` - อ่านข้อมูลตำแหน่งแบบรวมหลายระบบดาวเทียม

**รูปแบบผลลัพธ์:**
```
[mode],[GPS-SVs],[GLONASS-SVs],[BEIDOU-SVs],[lat],[N/S],[lon],[E/W],[date],[UTC-time],[alt],[speed],[course],[PDOP],[HDOP],[VDOP]
```

### `gnss_test`
**คำอธิบาย:** เปิด/ปิดโหมดทดสอบ GNSS
**การใช้งาน:** 
- `gnss_test -e` หรือ `gnss_test --enable` - เปิดโหมดทดสอบ
- `gnss_test -d` หรือ `gnss_test --disable` - ปิดโหมดทดสอบ
**คำสั่ง AT ที่ใช้:**
- `AT+CGPSFTM=1` - เปิดโหมดทดสอบ (สตรีม GxGSV ต่อเนื่อง)
- `AT+CGPSFTM=0` - ปิดโหมดทดสอบ

### `gnss_hotstart`
**คำอธิบาย:** ทำ GNSS hot start (ใช้ข้อมูลล่าสุดเพื่อจับดาวเร็วที่สุด)
**การใช้งาน:** `gnss_hotstart`
**คำสั่ง AT ที่ใช้:**
- `AT+CGPSHOT` - สั่ง hot start

### `gnss_coldstart`
**คำอธิบาย:** ทำ GNSS cold start (ล้างข้อมูลช่วยและเริ่มใหม่หมด)
**การใช้งาน:** `gnss_coldstart`
**คำสั่ง AT ที่ใช้:**
- `AT+CGPSCOLD` - สั่ง cold start

### `gnss_agps`
**คำอธิบาย:** เปิดใช้งาน AGPS เพื่อช่วยให้จับดาวเร็วขึ้น
**การใช้งาน:** `gnss_agps`
**คำสั่ง AT ที่ใช้:**
- `AT+CAGPS` - ขอข้อมูล AGPS จากเซิร์ฟเวอร์

## ขั้นตอนการทดสอบ

### 1. การเริ่มต้นพื้นฐาน
```bash
gnss_init         # เริ่มต้นระบบ GNSS
gnss_start        # เริ่มการระบุตำแหน่ง
```

### 2. ตรวจสอบตำแหน่ง
```bash
gnss_location     # อ่านตำแหน่งปัจจุบัน
```

### 3. การทดสอบขั้นสูง
```bash
gnss_agps         # เปิด AGPS เพื่อช่วยจับดาว
gnss_test -e      # เปิดโหมดทดสอบเพื่อดูดาวเทียม
gnss_hotstart     # ทำ hot start สำหรับการจับดาวเร็ว
gnss_coldstart    # ทำ cold start เมื่อต้องการเริ่มใหม่
```

### 4. ปิดระบบ
```bash
gnss_test -d      # ปิดโหมดทดสอบ (ถ้าเปิดไว้)
gnss_stop         # หยุดการทำงานของ GNSS
```

## หมายเหตุสำคัญ

1. **ข้อกำหนดเบื้องต้น:** คำสั่งเหล่านี้ต้องการให้ cellular module เริ่มต้นแล้ว ต้องเชื่อมต่อเครือข่าย cellular ก่อนใช้งาน

2. **การใช้งานคอนโซล:** คำสั่งจะพร้อมใช้งานเฉพาะในการ boot ครั้งแรก (xWakeUpCounter == 0) เท่านั้น

3. **NMEA Output:** หลังจากใช้ `gnss_start` ข้อมูล NMEA จะถูกส่งไปยัง UART port โดยตรง

4. **การจับดาวเทียม:** ใน environment ในร่มหรือพื้นที่ที่มีสัญญาณดาวเทียมอ่อน อาจต้องใช้เวลานานกว่าจะได้ fix

5. **Test Mode:** เมื่อเปิดโหมดทดสอบจะเห็นข้อมูล GxGSV (GPS, GLONASS, BeiDou) แสดงรายละเอียดดาวเทียมที่มองเห็น

## ตัวอย่างการใช้งาน

```bash
# เริ่มต้นและทดสอบพื้นฐาน
gnss_init
gnss_agps
gnss_start

# รอสักครู่แล้วตรวจสอบตำแหน่ง
gnss_location

# หากไม่ได้ fix ลอง cold start
gnss_coldstart

# ตรวจสอบการมองเห็นดาวเทียม
gnss_test -e

# หยุดการทำงาน
gnss_test -d
gnss_stop
```

## การแก้ไขปัญหา

- **"Cellular module not initialized":** ต้องรอให้ระบบเชื่อมต่อ cellular network ก่อน
- **"No GNSS fix available":** ตรวจสอบเสาอากาศและสถานที่ท้องฟ้าเปิด ลองใช้ `gnss_agps` และ `gnss_coldstart`
- **"Timeout waiting for GNSS power on":** อาจมีปัญหาฮาร์ดแวร์หรือการเชื่อมต่อ
