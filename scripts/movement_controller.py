#!/usr/bin/env python3
import time
import rclpy
import serial

import serial.tools.list_ports

from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial.serialutil import SerialException

"""
Пример сообщения
    Бит обозначающий устройство 0 - пк, 1 - педали, 2 - кпп, 3 - руль
    Бит обозначающий тип команды 0 - 7
    4 Бита со значениями
"""


COMP_MODULE = '0'
PEDAL_MODULE = '1'
TRANSMISSION_MODULE = '2'
STEERING_MODULE = '3'


PEDAL_MSGS = {
    'Current Speed': '0',                    # Текущая скорость автомобиля от блока управления
    'Current Clutch': '1',                   # Текущее состояние педали сцепления от блока управления
    'Current Break': '2',                    # Текущее состояние педали тормоза от блока управления

    'Required Clutch': '3',                  # Требуемое состояние сцепления от компьютера
    'Required Break': '4',                   # Требуемое состояние тормоза от компьютера

    'Pedal Connection Error': '5',           # Ошибка последовательного соединения
    'Pedal Data Error': '6',                 # Ошибка целостности данных последовательного соединения
}

STEERING_MSGS = {
    'Current Steering': '0',                 # Текущий угол поворота колес от блока управления
    'Current Ignition': '1',                 # Текущее состояние зажигания от блока управления

    'Required Steering': '2',                # Требуемый угол поворота колес от компьютера

    'Ignition (EPS) Error': '3',             # Двигатель не заведен, эур не работает
    'Encoder Connection Error': '4',         # Потеря сигнала от энкодера руля
    'Steering Timeout Error': '5',           # Руль не выходит в заданное положение за время таймаута
    'Steering Jump Over Error': '6',         # Прокрутка энкодера больше одного оборота

    'Steering Connection Error': '7',        # Ошибка последовательного соединения
    'Steering Data Error': '8',              # Ошибка целостности данных последовательного соединения
}

TRANSMISSION_MSGS = {
    'Current Gear': '0',                     # Текущая включенная передача от блока управления

    'Required Gear': '1',                    # Требуемая передача от компьютера

    'FPotentiometer Connection Error': '2',  # Потеря сигнала потенциометра актуатора переключения вперед-назад
    'FPotentiometer Range Error': '3',       # Выход из допустимых диапазонов потенциометра актуатора переключения вперед-назад
    'FTransmission Activation Error': '4',   # Ошибка включения передачи актуатора переключения вперед-назад

    'SPotentiometer Connection Error': '5',  # Потеря сигнала потенциометра актуатора переключения влево-вправо
    'SPotentiometer Range Error': '6',       # Выход из допустимых диапазонов потенциометра актуатора переключения влево-вправо
    'STransmission Activation Error': '7',   # Ошибка включения передачи актуатора переключения влево-вправо

    'Transmission Connection Error': '8',    # Ошибка последовательного соединения
    'Transmission Data Error': '9',          # Ошибка целостности данных последовательного соединения
}


PRESS_PEDAL = "1.00"
RELEASE_PEDAL = "0.00"

ON_IGNITION = "1.00"
OFF_IGNITION = "0.00"

FIRST_GEAR = "1.00"
REVERSE_GEAR = "9.00"
NEUTRAL_GEAR = "0.00"


class MovementController(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('reset_errors', False)
        self.declare_parameter('steering_serial_port', '1-12:1.0')
        self.declare_parameter('pedal_serial_port', '1-1')
        self.declare_parameter('transmission_serial_port', '1-3')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.01)

        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.steering_port = self.get_parameter('steering_serial_port').get_parameter_value().string_value
        self.pedal_port = self.get_parameter('pedal_serial_port').get_parameter_value().string_value
        self.transmission_port = self.get_parameter('transmission_serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value

        self.logger = self.get_logger()
        self.logger.set_level(50)
        self.errors = []

        self.reconnect_steering()
        self.reconnect_pedal()
        self.reconnect_transmission()

        self.cmdSub = self.create_subscription(Twist,
                                               cmd_topic,
                                               self.cmd_callback,
                                               10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)
        self.clutchTimer = self.create_timer(1 / frequency, self.clutch_callback)
        self.breakTimer = self.create_timer(1 / frequency, self.break_callback)
        self.steeringTimer = self.create_timer(1 / frequency, self.steering_callback)
        self.transmissionTimer = self.create_timer(1 / frequency, self.transmission_callback)

        self.clutchTimer.cancel()
        self.breakTimer.cancel()

        self.serialTimer = self.create_timer(1 / frequency, self.serial_callback)

        self.currentSpeed = 0.

        self.currentBrakeState = 0
        self.currentClutchState = 0
        self.currentSteeringAngle = 0
        self.currentTransmissionState = 0
        self.currentIgnitionState = 0

        self.reqBreakState = 0
        self.reqClutchState = 0
        self.reqSteeringAngle = 0
        self.reqTransmissionState = 0

        self.cmdData = Twist()

    def cmd_callback(self, msg):
        self.cmdData = msg

    def timer_callback(self):
        if self.get_parameter('reset_errors').get_parameter_value().bool_value:
            self.reset_errors()
            self.set_parameters([rclpy.Parameter('reset_errors', rclpy.Parameter.Type.BOOL, False)])

        if not self.errors:
            if self.currentIgnitionState:
                speed = self.cmdData.linear.x
                steer = self.cmdData.angular.z

                # Устанавливаем требуемый угол поворота рулевого колеса
                # Отправка требуемого угла производится в соответствующем таймере
                self.reqSteeringAngle = steer

                if speed > 0:
                    self.reqTransmissionState = 1
                elif speed < 0:
                    self.reqTransmissionState = -1
                else:
                    self.reqTransmissionState = 0

                if self.currentClutchState == 0 and self.currentBrakeState == 0 and self.reqTransmissionState != self.currentTransmissionState:
                    self.press_clutch()
                    self.press_break()
                if self.currentClutchState == 1 and self.currentBrakeState == 1 and self.reqTransmissionState != self.currentTransmissionState:
                    # Устанавливаем требуемую передачу для включения
                    # Отправка требуемой передачи производится в соответствующем таймере
                    pass
                if self.currentClutchState == 1 and self.currentBrakeState == 1 and self.reqTransmissionState == self.currentTransmissionState:
                    self.release_break()
                    self.release_clutch()
            else:
                self.press_clutch()
                self.press_break()
                self.logger.info("The car is not started! Waiting ignition...")
                time.sleep(3)
        else:
            self.reqTransmissionState = 0.
            self.reqSteeringAngle = 0.
            self.logger.error("Check errors!")
        print(self.errors)
        self.reset_errors()

    def transmission_callback(self):
        pass

    def steering_callback(self):
        try:
            if self.steeringSerial.is_open:
                msg = COMP_MODULE + STEERING_MSGS.get('Required Steering') + f"{round(self.reqSteeringAngle, 2)}"
                self.steeringSerial.write(bytes(msg, 'utf-8'))
            else:
                self.logger.error('Steering Module Serial Port is not available!')
                self.reconnect_steering()
        except AttributeError:
            self.reconnect_steering()

    def clutch_callback(self):
        # Отправить требуемое состояние сцепления
        try:
            if self.pedalSerial.is_open:
                if self.reqClutchState == 1:
                    msg = COMP_MODULE + PEDAL_MSGS.get('Required Clutch') + PRESS_PEDAL
                else:
                    msg = COMP_MODULE + PEDAL_MSGS.get('Required Clutch') + RELEASE_PEDAL
                self.pedalSerial.write(bytes(msg, 'utf-8'))
            else:
                self.logger.error('Pedal Module Serial Port is not available!')
                self.insert_error(PEDAL_MSGS, 5)
                self.reconnect_pedal()
            if self.currentClutchState == self.reqClutchState:
                txt = "Pressed" if self.currentClutchState == 1 else "Released"
                self.logger.info(f"Clutch is {txt}")
                self.clutchTimer.cancel()
        except AttributeError:
            self.reconnect_pedal()

    def break_callback(self):
        # Отправить требуемое состояние тормоза
        try:
            if self.pedalSerial.is_open:
                if self.reqBreakState == 1:
                    msg = COMP_MODULE + PEDAL_MSGS.get('Required Break') + PRESS_PEDAL
                else:
                    msg = COMP_MODULE + PEDAL_MSGS.get('Required Break') + RELEASE_PEDAL
                self.pedalSerial.write(bytes(msg, 'utf-8'))
            else:
                self.logger.error('Pedal Module Serial Port is not available!')
                self.insert_error(PEDAL_MSGS, 5)
                self.reconnect_pedal()
            if self.currentBrakeState == self.reqBreakState:
                txt = "Pressed" if self.currentBrakeState == 1 else "Released"
                self.logger.info(f"Break is {txt}")
                self.breakTimer.cancel()
        except AttributeError:
            self.reconnect_pedal()

    def serial_callback(self):
        # Получение всех параметров автомобиля
        try:
            # Получение данных с модуля педалей
            pedal_msg = self.pedalSerial.readline()
            try:
                pedal_msg = pedal_msg.decode('utf-8')
                if pedal_msg[-2:] == '\r\n':
                    pedal_msg = pedal_msg[:-2]
                    if len(pedal_msg) == 6:
                        if pedal_msg[0] == PEDAL_MODULE:
                            if pedal_msg[1] == '0':
                                self.currentSpeed = float(pedal_msg[3:])
                            if pedal_msg[1] == '1':
                                self.currentClutchState = int(float(pedal_msg[2:]))
                            if pedal_msg[1] == '2':
                                self.currentBrakeState = int(float(pedal_msg[2:]))
            except Exception as e:
                self.logger.error(f'Pedal Module Data Error: {e}')
                self.insert_error(PEDAL_MSGS, 6)
        except (SerialException, AttributeError):
            self.logger.error('Pedal Module Serial Port is not available!')
            self.insert_error(PEDAL_MSGS, 5)

        # try:
            # Получение данных с модуля трансмиссии
            # transmission_msg = self.transmissionSerial.readline()
        try:
            # Получение данных с модуля подруливания
            steering_msg = self.steeringSerial.readline()
            try:
                steering_msg = steering_msg.decode('utf-8')
                if steering_msg[-2:] == '\r\n':
                    steering_msg = steering_msg[:-2]
                    if len(steering_msg) == 6:
                        if steering_msg[0] == STEERING_MODULE:
                            if steering_msg[1] == '0':
                                self.currentSteeringAngle = float(steering_msg[3:])
                            if steering_msg[1] == '1':
                                self.currentIgnitionState = int(float(steering_msg[2:]))
            except Exception as e:
                self.logger.error(f'Steering Module Data Error: {e}')
                self.insert_error(STEERING_MSGS, 8)
        except (SerialException, AttributeError):
            self.logger.error(f'Steering Module Serial Port is not available!')
            self.insert_error(STEERING_MSGS, 7)


    def press_clutch(self):
        if self.currentClutchState == 0:
            if self.clutchTimer.is_canceled():
                self.reqClutchState = 1
                self.clutchTimer.reset()

    def press_break(self):
        if self.currentBrakeState == 0:
            if self.breakTimer.is_canceled():
                self.reqBreakState = 1
                self.breakTimer.reset()

    def release_clutch(self):
        if self.currentClutchState == 1:
            if self.clutchTimer.is_canceled():
                self.reqClutchState = 0
                self.clutchTimer.reset()

    def release_break(self):
        if self.currentBrakeState == 1:
            if self.breakTimer.is_canceled():
                self.reqBreakState = 0
                self.breakTimer.reset()

    def insert_error(self, errs, num):
        err = list(errs.keys())[num]
        self.errors.append(err)
        self.errors = list(set(self.errors))

    def reset_errors(self):
        self.errors = []

    def find_port(self, location):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.location == location:
                return port.device

    def reconnect_steering(self):
        port = self.find_port(self.steering_port)
        try:
            self.steeringSerial = serial.Serial(port, self.baudrate, timeout=self.timeout)
        except SerialException:
            self.logger.error('Steering Module Serial Port is not available!')
            self.insert_error(STEERING_MSGS, 7)

    def reconnect_pedal(self):
        port = self.find_port(self.pedal_port)
        try:
            self.pedalSerial = serial.Serial(port, self.baudrate, timeout=self.timeout)
        except SerialException:
            self.logger.error('Pedal Module Serial Port is not available!')
            self.insert_error(PEDAL_MSGS, 5)

    def reconnect_transmission(self):
        port = self.transmission_port
        try:
            self.transmissionSerial = serial.Serial(port, self.baudrate, timeout=self.timeout)
        except SerialException:
            self.logger.error('Transmission Module Serial Port is not available!')
            self.insert_error(TRANSMISSION_MSGS, 8)


def main():
    rclpy.init()
    node = MovementController("movement_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
