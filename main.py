from dronekit_sitl import SITL
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import time
import math
from pymavlink import mavutil
from geopy.distance import geodesic

# === КОНФІГУРАЦІЯ СИМУЛЯТОРА ===
# Завантаження та запуск SITL з параметрами

sitl = SITL()
sitl.download('copter', 'stable', verbose=True) 
sitl_args = [
    '-I0', 
    '--model', 'quad',
    '--home=50.450739,30.461242,161,0', # Точка А = Точка зльоту = Точка "Home":  50.450739, 30.461242
    '--speedup=100'  # Швидкість
]
sitl.launch(sitl_args, await_ready=True, restart=True)


# Підключення до симулятора через TCP
connection_string = 'tcp:127.0.0.1:5760'
print(f"Підключення до дрону на {connection_string}")
vehicle = connect(connection_string, wait_ready=True)
print(f"Підключення прошло успішно")


# Отримання коректних координат
def wait_for_gps():
    while vehicle.location.global_relative_frame.lat is None or vehicle.location.global_relative_frame.lon is None:
        print("⚠ Ожидание GPS...", end='\r')
        time.sleep(1)
    print(f"✅ GPS готовий! Координати: {vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon}")


def wait_until_fully_armable():
    """"Перевірка"""
    print("🔍 Перевірка систем перед запуском...")
    last_print = time.time()
    while True:
        checks = [
            vehicle.is_armable,
            vehicle.gps_0.fix_type >= 3,
            vehicle.battery.level > 10,
            vehicle.ekf_ok
        ]
        if all(checks):
            break
        
        if time.time() - last_print > 1:
            print(f"Очікування: GPS={vehicle.gps_0.fix_type}, Батарея={vehicle.battery.level}%, EKF={'OK' if vehicle.ekf_ok else 'WAIT'}", end='\r')
            last_print = time.time()
        time.sleep(0.1)
    print("\n✅ Всі системи готові до запуску!")


def arm_and_takeoff(target_altitude):
    """Процедура підйому на задану висоту с тестами"""
    wait_until_fully_armable()
    
    print("🔄 Активація режиму ALT_HOLD")
    vehicle.mode = VehicleMode("ALT_HOLD")
    time.sleep(1)
    
    print("⚡ Запуск моторів..")
    vehicle.armed = True
    
    start_time = time.time()
    last_print = start_time
    while not vehicle.armed:
        if time.time() - start_time > 10:
            raise Exception("Помилка! Перевірте режим та блокування")
        
        if time.time() - last_print > 1:
            print("⏳ Очікування запуску моторів...", end='\r')
            last_print = time.time()
        time.sleep(0.1)
    print("\n✅ Мотори активовані!")


def alt_hold_takeoff(target_alt):
    """Підйом на задану висоту в режимі ALT_HOLD"""
    print(f"🛫 Підйом до {target_alt} м в ALT_HOLD")
    
    start_alt = vehicle.location.global_relative_frame.alt or 0
    last_alt = start_alt
    while True:
        current_alt = vehicle.location.global_relative_frame.alt or 0
        
        if current_alt >= target_alt:
            break

        # Пропорційний контроль висоти
        alt_error = target_alt - current_alt
        throttle = 1500 + int(70 * min(1, alt_error/3)) 
        
        send_rc_control(1500, 1500, throttle, 1500)
        
        if abs(current_alt - last_alt) > 0.5 or current_alt == start_alt:
            print(f"Висота: {current_alt:.1f}/{target_alt} м", end='\r')
            last_alt = current_alt
        time.sleep(0.1)
    
    send_rc_control(1500, 1500, 1500, 1500)
    print(f"\n🛬 Досягнуто цільову висоту: {current_alt:.1f} м")


def send_rc_control(roll, pitch, throttle, yaw):
    """Надсилання RC команд управління"""
    msg = vehicle.message_factory.rc_channels_override_encode(
        0, 0,  # target_system, target_component
        roll, pitch, throttle, yaw,
        0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def get_bearing_to_point(lat1, lon1, lat2, lon2):
    """Розрахунок азимуту між двома точками"""
    dLon = math.radians(lon2 - lon1)
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def move_to_location(target_lat, target_lon):
    print(f"🔄 Рух до точки  {target_lat}, {target_lon}")
    
    target_alt = vehicle.location.global_relative_frame.alt
    HIGH_SPEED_DISTANCE = 20  # Дистанція початку уповільнення
    PRECISION_DISTANCE = 10   # Дистанція вважається досягнутою
    MIN_SPEED = 1520          
    BASE_SPEED = 1650         
    
    KP = 1.8  # Коэффициент поворота

    # Основний цикл навігації
    while True:
        current = vehicle.location.global_relative_frame
        distance = geodesic((current.lat, current.lon), (target_lat, target_lon)).meters
        current_speed = math.sqrt(vehicle.velocity[0]**2 + vehicle.velocity[1]**2)
        current_alt = current.alt or 0
        
        # Расчет направления
        desired_heading = get_bearing_to_point(current.lat, current.lon, target_lat, target_lon)
        heading_error = (desired_heading - vehicle.heading + 540) % 360 - 180
        
        # Управление скоростью (3 фазы)
        if distance > HIGH_SPEED_DISTANCE:
            # Полная скорость на дальних дистанциях
            speed = BASE_SPEED
        elif distance > 5:
            # Плавное замедление при приближении
            speed = MIN_SPEED + int((BASE_SPEED - MIN_SPEED) * (distance-5)/(HIGH_SPEED_DISTANCE-5))
        else:
            # Почти остановка вблизи цели
            speed = 1500
        
        # Управление поворотом
        roll_correction = KP * heading_error
        roll = 1500 + int(max(-400, min(400, roll_correction)))
        
        # Контроль висоти
        alt_error = target_alt - current_alt
        throttle = 1500 + int(max(-150, min(150, alt_error * 15)))
        
        # Отправка команд
        send_rc_control(roll, speed, throttle, 1500)
        
        # Умови завершення
        if distance < PRECISION_DISTANCE and current_speed < 1.0:
            print("\n🎯 Точка майже досягнута (10м), завершення...")
            break
            
        print(f"Дистанція: {distance:5.1f} м | Швидкість: {current_speed:3.1f} м/с | Висота: {current_alt:5.1f} м", end='\r')
        time.sleep(0.1)
    
    # Точная стабилизация перед завершением
    print("🔧 Точная стабилизация...")
    for i in range(50, 0, -1):  # 5 секунд стабилизации
        send_rc_control(1500, 1500, 1500, 1500)
        print(f"Стабилизация... {i/10:.1f}с", end='\r')
        time.sleep(0.1)
    
    final_pos = vehicle.location.global_relative_frame
    accuracy = geodesic((final_pos.lat, final_pos.lon), (target_lat, target_lon)).meters
    print(f"\n✅ Позиціювання завершено! Похибка: {accuracy:.1f} м")


def set_yaw(target_heading):
    """Поворот на заданий азимут з утриманням позиції"""

    print(f"🔄 Поворот на курс {target_heading}°...")
    
    target_alt = vehicle.location.global_relative_frame.alt
    YAW_SPEED = 60  # Скорость поворота (PWM)
    last_heading = vehicle.heading
    
    while True:
        current_heading = vehicle.heading
        heading_error = (target_heading - current_heading + 540) % 360 - 180
        
        # Условие завершения
        if abs(heading_error) < 3:
            break
        
        # Управление поворотом
        yaw_value = 1500 + int(YAW_SPEED * (1 if heading_error > 0 else -1))
        
        # Контроль высоты и позиции
        current_alt = vehicle.location.global_relative_frame.alt or 0
        alt_error = target_alt - current_alt
        throttle = 1500 + int(max(-200, min(200, alt_error * 20)))
        
        # Небольшие корректировки положения
        roll = 1500 + int(30 * math.sin(time.time()))  # Микрокоррекции
        pitch = 1500  # Нейтральное положение
        
        send_rc_control(roll, pitch, throttle, yaw_value)
        
        if abs(current_heading - last_heading) > 1:
            print(f"Текущий курс: {current_heading:.1f}° → Ціль: {target_heading}° | Помилка: {heading_error:.1f}°", end='\r')
            last_heading = current_heading
        time.sleep(0.1)
    
    send_rc_control(1500, 1500, 1500, 1500)
    print(f"\n✅ Поворот завершен! Финальный курс: {vehicle.heading:.1f}°")


B_LAT, B_LON = 50.443326, 30.448078
TARGET_ALTITUDE = 100

try:
    # Основная последовательность
    wait_for_gps()
    arm_and_takeoff(TARGET_ALTITUDE)
    alt_hold_takeoff(TARGET_ALTITUDE)
    move_to_location(B_LAT, B_LON)
    set_yaw(350)
    wait_for_gps()
    print("✅ Місію виконано! Посадка...")
    vehicle.mode = VehicleMode("LAND")

except Exception as e:
    print(f"❌ Критична помилка: {str(e)}")
    vehicle.mode = VehicleMode("LAND")

finally:
    vehicle.close()
    sitl.stop()