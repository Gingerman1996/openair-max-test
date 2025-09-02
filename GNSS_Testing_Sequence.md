# GNSS Testing Sequence

## การทดสอบ GNSS หลังจาก SIM Card พร้อม

### ขั้นตอนที่ 1: รอให้ Cellular Module พร้อม
รอให้เห็นข้อความที่บ่งบอกว่า cellular module เชื่อมต่อแล้ว หรือลองทดสอบด้วยคำสั่ง:

```bash
help  # เพื่อดูคำสั่งทั้งหมด
```

### ขั้นตอนที่ 2: เริ่มต้น GNSS Module
```bash
gnss_init
```
คำสั่งนี้จะ:
- เปิดใช้งาน GNSS module ด้วย AT+CGNSSPWR=1
- ตั้งค่าโหมด satellite constellation (GPS+GLONASS+Galileo+BeiDou)
- เปิดใช้งาน NMEA output

### ขั้นตอนที่ 3: ทำ Cold Start (ตามที่ขอ)
```bash
gnss_coldstart
```
คำสั่งนี้จะ:
- ล้างข้อมูล assistance ทั้งหมด
- รีเซ็ต GNSS เพื่อเริ่มใหม่ทั้งหมด
- ใช้เวลานานกว่าในการ acquire สัญญาณ แต่จะแม่นยำกว่า

### ขั้นตอนที่ 4: เริ่มการหาพิกัด
```bash
gnss_start
```
คำสั่งนี้จะ:
- เริ่มการค้นหา satellite และหาพิกัด
- เปิดใช้ NMEA streaming ไปยัง UART port

### ขั้นตอนที่ 5: ตรวจสอบตำแหน่งและเวลา
```bash
gnss_location
```
คำสั่งนี้จะแสดง:
- พิกัดปัจจุบัน (latitude, longitude)
- ความสูง (altitude)
- เวลา UTC
- ความเร็ว
- ทิศทาง
- จำนวน satellite ที่เชื่อมต่อ
- ค่า PDOP, HDOP, VDOP (ความแม่นยำ)

### ขั้นตอนที่ 6: ตรวจสอบสัญญาณ GNSS (ตามที่ขอ)
```bash
gnss_test -e
```
คำสั่งนี้จะ:
- เปิดโหมดทดสอบสัญญาณ satellite
- แสดงข้อมูล satellite visibility (GxGSV sentences)
- แสดงสถานะและความแรงสัญญาณของ satellite แต่ละดวง

เพื่อปิดโหมดทดสอบ:
```bash
gnss_test -d
```

### ขั้นตอนที่ 7: เปิดใช้ AGPS (ช่วยเพิ่มความเร็ว)
```bash
gnss_agps
```
คำสั่งนี้จะ:
- เปิดใช้ Assisted GPS
- ดาวน์โหลดข้อมูล assistance จากเครือข่าย
- ช่วยให้หาสัญญาณ satellite เร็วขึ้น

## หมายเหตุการทดสอบ

### เวลาในการหาสัญญาณ:
- **Cold Start**: 30-60 วินาที (ถ้าสัญญาณดี)
- **Hot Start**: 5-15 วินาที
- **AGPS**: ช่วยลดเวลาลงได้ 50-80%

### การตรวจสอบผลลัพธ์:
1. **Fix Status**: ตรวจสอบว่าได้ fix หรือยัง (โหมด 3D fix คือ GPS ทำงานปกติ)
2. **Satellite Count**: ควรเห็น satellite อย่างน้อย 4 ดวงสำหรับ 3D fix
3. **HDOP**: ค่าน้อยกว่า 2.0 ถือว่าแม่นยำดี
4. **Location**: พิกัดควรสมเหตุสมผลกับตำแหน่งปัจจุบัน

### คำสั่งเพิ่มเติม:
```bash
gnss_hotstart   # Hot start (ใช้ข้อมูลเก่า)
gnss_stop       # หยุดการทำงาน GNSS
```

## การติดตามผลลัพธ์

ระหว่างการทดสอบ คุณจะเห็น:
1. ข้อความ initialization
2. ข้อความ satellite acquisition 
3. ข้อมูล NMEA sentences (ถ้าเปิด test mode)
4. ข้อมูลตำแหน่งและเวลา

ถ้าอยู่ในอาคารหรือพื้นที่ที่สัญญาณ GPS อ่อน อาจต้องใช้เวลานานขึ้นหรือไม่สามารถหาสัญญาณได้
