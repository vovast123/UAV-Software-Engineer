import numpy as np

def calculate_image_center_coordinates(control_lat, control_lon, azimuth,point_x, point_y, center_x, center_y, scale):
    # Зсув пікселів відносно центру зображення
    dx = center_x - point_x
    dy = center_y - point_y
    
    # Перетворюємо пікселі в метри
    dx_m = dx * scale
    dy_m = dy * scale
    
    # Перетворюємо азимут у радіани
    azimuth_rad = np.radians(azimuth)
    
    # Обчислюємо зміщення в глобальній системі координат
    delta_north = -dy_m * np.cos(azimuth_rad) + dx_m * np.sin(azimuth_rad)
    delta_east = dy_m * np.sin(azimuth_rad) + dx_m * np.cos(azimuth_rad)
    
    # Приблизне обчислення координат (грубо, без урахування кривизни Землі)
    lat_per_meter = 1 / 111111  # 1 градус широти ≈ 111.111 км
    lon_per_meter = 1 / (111111 * np.cos(np.radians(control_lat)))
    
    center_lat = control_lat + delta_north * lat_per_meter
    center_lon = control_lon + delta_east * lon_per_meter
    
    return center_lat, center_lon

# Вхідні дані
control_lat = 50.603694  # Широта контрольної точки
control_lon = 30.650625  # Довгота контрольної точки
azimuth = 335  # Азимут "вгору" за зображенням
point_x = 558  # X-координата контрольної точки на зображенні
point_y = 328  # Y-координата контрольної точки на зображенні
center_x = 320  # X-координата центру зображення
center_y = 256  # Y-координата центру зображення
scale = 0.38  # 1 піксель = 0.38 м

# Обчислюємо координати центру зображення
center_coordinates = calculate_image_center_coordinates(control_lat, control_lon, azimuth,point_x, point_y, center_x, center_y, scale)

print(f"Координати центру зображення: {float(center_coordinates[0])}, {float(center_coordinates[1])}")
