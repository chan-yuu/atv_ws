import can
import cantools
import rospy
import math
from datetime import datetime, timedelta
import time
import pyproj
from car_interfaces.msg import GpsImuInterface

class OutOfRangeError(ValueError):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

class GpsImu:
    def __init__(self, dbc_file, can_channel):
        self.state_flag = 0
        self.state_flag1 = 0
        self.k_s = 0
        self.flag_lat = False
        self.flag_lon = False
        self.now_speed = 0
        self.GPS_MSG = GpsImuInterface()

        self.db = cantools.db.load_file(dbc_file)
        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            print("CAN已连接")
        except:
            print("CAN未连接")

    def call_back_VCU(self, VCU_MSG):
        pass
        # self.now_speed = VCU_MSG.Car_Speed
    #     print("now_speed",self.now_speed)

    def publish_data(self, pub):
        message = self.bus.recv()
        # print(message)
        self.GPS_MSG.header.stamp = rospy.Time.now()

        if message.arbitration_id == 0x32A:
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            self.GPS_MSG.AngleHeading = self.angle_2_angle(decoded_message["AngleHeading"])
            self.GPS_MSG.pitch = decoded_message["AnglePitch"]
            self.GPS_MSG.roll = decoded_message["AngleRoll"]
            self.GPS_MSG.yaw = self.GPS_MSG.AngleHeading

        elif message.arbitration_id == 0x323:
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            self.GPS_MSG.system_state = decoded_message["system_state"]
            self.GPS_MSG.satellite_status = decoded_message["satellite_status"]

        elif message.arbitration_id == 0x32D:
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            self.GPS_MSG.lon = decoded_message["PosLon2"]
            self.GPS_MSG.PosLon = self.GPS_MSG.lon
            self.flag_lon = True

        elif message.arbitration_id == 0x32E:
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            self.GPS_MSG.lat = decoded_message["PosLat2"]
            self.GPS_MSG.PosLat = self.GPS_MSG.lat
            self.flag_lat = True

        elif message.arbitration_id == 0x327                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 :
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            self.GPS_MSG.VelU = decoded_message["VelU"]
            self.GPS_MSG.VelN = decoded_message["VelN"]
            self.GPS_MSG.VelE = decoded_message["VelE"]
            self.GPS_MSG.Vel = decoded_message["Vel"]
            self.now_speed = self.GPS_MSG.Vel
            # print("now_speed",self.now_speed)

        if self.flag_lon and self.flag_lat:
            self.GPS_MSG.posX = self.from_latlon(self.GPS_MSG.lat, self.GPS_MSG.lon)[0]
            self.GPS_MSG.posY = self.from_latlon(self.GPS_MSG.lat, self.GPS_MSG.lon)[1]

            # x = self.convert_to_utm(self.GPS_MSG.lat, self.GPS_MSG.lon)[0]

            # self.GPS_MSG.posX = self.convert_to_utm(self.GPS_MSG.lat, self.GPS_MSG.lon)[0]
            # self.GPS_MSG.posY = self.convert_to_utm(self.GPS_MSG.lat, self.GPS_MSG.lon)[1]

        # zone = self.convert_to_utm(self.GPS_MSG.lat, self.GPS_MSG.lon)[2]
        # zone_from = self.from_latlon(self.GPS_MSG.lat, self.GPS_MSG.lon)[2]
        # print("self.GPS_MSG.posX",self.GPS_MSG.posX)
        # print("x",x-self.GPS_MSG.posX,"zone", zone, zone_from)

            self.GPS_MSG.x = self.GPS_MSG.posX
            self.GPS_MSG.y = self.GPS_MSG.posY

            self.flag_lat = False
            self.flag_lon = False
            # self.GPS_MSG.Vel = self.now_speed
            pub.publish(self.GPS_MSG)

    def angle_2_angle(self, angle):
        angle -= 90
        while angle < -180:
            angle += 360
        while angle > 180:
            angle -= 360
        return -angle

    def calculate_utm_zone(self, longitude):
        zone = int((longitude + 180) / 6) + 1
        return zone



    # test 6.11 不再使用from_latlon，直接调库，同时注意一下调用的时间问题
    # def convert_to_utm(self, latitude, longitude):
    #     zone = self.calculate_utm_zone(longitude)

    #     utm_projection = pyproj(proj='utm', zone=zone, ellps='WGS84')
    #     utm_easting, utm_northing = utm_projection(longitude, latitude)

    #     return utm_easting, utm_northing, zone

    # def convert_to_utm(self, latitude, longitude):
    #     wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
    #     utm_zone = (math.floor((longitude + 180) / 6) % 60) + 1
    #     utm = pyproj.CRS(f'+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs')
    #     transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
    #     utm_x, utm_y = transformer.transform(longitude, latitude)
    #     return utm_x, utm_y, utm_zone
    # 已知经纬度求zone和utm坐标
    def convert_to_utm(self, latitude, longitude):
        wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
        utm_zone = (math.floor((longitude + 180) / 6) % 60) + 1
        utm_epsg_code = 32600 + utm_zone  # Assuming northern hemisphere; for southern use 32700 + zone
        utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
        transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
        utm_x, utm_y = transformer.transform(longitude, latitude)
        return utm_x, utm_y, utm_zone
    
    # 已知zone和utm坐标求经纬度
    def convert_from_utm(self, utm_x, utm_y, utm_zone, northern_hemisphere=True):
        utm_epsg_code = 32600 + utm_zone if northern_hemisphere else 32700 + utm_zone
        utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
        wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
        transformer = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
        longitude, latitude = transformer.transform(utm_x, utm_y)
        return latitude, longitude

    def from_latlon(self, latitude, longitude, force_zone_number=None):
        K0 = 0.9996
        E = 0.00669438
        E_P2 = E / (1.0 - E)
        R = 6378137
        if not -80.0 <= latitude <= 84.0:
            raise OutOfRangeError('latitude out of range (must be between 80 deg S and 84 deg N)')
        if not -180.0 <= longitude <= 180.0:
            raise OutOfRangeError('longitude out of range (must be between 180 deg W and 180 deg E)')

        lat_rad = math.radians(latitude)
        lat_sin = math.sin(lat_rad)
        lat_cos = math.cos(lat_rad)
        lat_tan = lat_sin / lat_cos
        lat_tan2 = lat_tan * lat_tan
        lat_tan4 = lat_tan2 * lat_tan2

        if force_zone_number is None:
            zone_number = self.latlon_to_zone_number(latitude, longitude)
        else:
            zone_number = force_zone_number

        zone_letter = self.latitude_to_zone_letter(latitude)
        lon_rad = math.radians(longitude)
        central_lon = self.zone_number_to_central_longitude(zone_number)
        central_lon_rad = math.radians(central_lon)
        n = R / math.sqrt(1 - E * lat_sin ** 2)
        c = E_P2 * lat_cos ** 2
        a = lat_cos * (lon_rad - central_lon_rad)
        a2 = a * a
        a3 = a2 * a
        a4 = a3 * a
        a5 = a4 * a
        a6 = a5 * a

        M1 = (1 - E / 4 - 3 * E ** 2 / 64 - 5 * E ** 3 / 256)
        M2 = (3 * E / 8 + 3 * E ** 2 / 32 + 45 * E ** 3 / 1024)
        M3 = (15 * E ** 2 / 256 + 45 * E ** 3 / 1024)
        M4 = (35 * E ** 3 / 3072)
        m = R * (M1 * lat_rad - M2 * math.sin(2 * lat_rad) + M3 * math.sin(4 * lat_rad) - M4 * math.sin(6 * lat_rad))

        easting = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
        northing = K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))

        if latitude < 0:
            northing += 10000000

        return easting, northing, zone_number, zone_letter

    def latlon_to_zone_number(self, latitude, longitude):
        if 56 <= latitude < 64 and 3 <= longitude < 12:
            return 32
        if 72 <= latitude <= 84 and longitude >= 0:
            if longitude <= 9:
                return 31
            elif longitude <= 21:
                return 33
            elif longitude <= 33:
                return 35
            elif longitude <= 42:
                return 37
        return int((longitude + 180) / 6) + 1

    def latitude_to_zone_letter(self, latitude):
        ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
        if -80 <= latitude <= 84:
            return ZONE_LETTERS[int(latitude + 80) >> 3]
        else:
            return None

    def zone_number_to_central_longitude(self, zone_number):
        return (zone_number - 1) * 6 - 180 + 3

    def gps_week_seconds_to_utc(self, gpsweek, gpsseconds, leapseconds=18):
        datetimeformat = "%Y-%m-%d %H:%M:%S.%f"
        gpsseconds = gpsseconds / 1000
        epoch = datetime.strptime("1980-01-06 08:00:00.000", datetimeformat)
        elapsed = timedelta(days=(gpsweek * 7), seconds=(gpsseconds - leapseconds))
        beijing_time = datetime.strftime(epoch + elapsed, datetimeformat)
        time_stamp = int((time.mktime(datetime.strptime(beijing_time, "%Y-%m-%d %H:%M:%S.%f").timetuple())))
        return beijing_time, time_stamp
