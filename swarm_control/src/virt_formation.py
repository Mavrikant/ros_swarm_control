#!/usr/bin/env python
# coding=utf8


import numpy as np
from swarm_contol.msg import FormationParam


def create_virtual_structure(n, lengthRTP, widthRTP, formation, alpha=0.0):
    """
    Функция виртуального построения
    Virtual_formation - массив координат расположения РТП в строю
    x0,y0 - координаты центра виртуального построения
    sizeBuild - размер построния (ширина Х глубина)
    :param n: количество РТП
    :param lengthRTP: длина РТП
    :param widthRTP: ширина РТП
    :param distance: расстояние между РТП
    :param typeFormation: тип построения, задается от 1 до 7:
                            0-Фаланга RANK=0
                            1-Каре (квадрат) SQUARE=1
                            2-Клин KLIN=2
                            3-Обратный клин REVERSE_KLIN=3
                            4-Колонна COLUMM=4
                            5-Эшелон ECHELON=5
                            6-Кольцо CIRCLE=6
    :param alpha: угол поворота виртуального построения
    :return:
    """

    safety_radius_rtp = np.sqrt((lengthRTP / 2) ** 2 + (widthRTP / 2) ** 2)

    sizeBuild = np.zeros((1, 2))
    Virtual_formation = np.zeros((n, 2))

    if formation.type == FormationParam.RANK and formation.type == FormationParam.NO_DATA:  # Фаланга
        r = formation.distance / 2 + safety_radius_rtp
        if n == 2:
            a = 2 * r * n
        else:
            a = (2 * r) * np.ceil(n / 2)

        if n > 4:
            b = 4 * r
        else:
            b = (2 * r) * np.ceil(n / 2)
        # sizeBuild = [a b];
        kx = np.fix((a / (2 * r)))  # количество РТП помещающихся по горизонтали
        ky = np.fix((b / (2 * r))) # количество РТП помещающихся по вертикали
        N = kx * ky  # максимальное количество роботов
        # n = 5;  #n = 12;  # количество роботов вводимые пользователем
        D = N - n  # разница между максимальным количеством
        #            роботов и необходимым, заданным пользователем
        if np.fix(D / kx) > 0:
            ky = ky - np.fix(D / kx)
            N = N - np.fix(D / kx) * kx
        dx = (a - kx * (2 * r)) / (kx + 1)  # расстояние между зонами безопасности РТП
        dy = (b - ky * (2 * r)) / (ky + 1)  # расстояние между зонами безопасности РТП
        y = (- b) / 2 - r  # начало отсчета по у
        k = 0
        for j in np.arange(1, ky + 1):
            y = y + dy + np.dot(2, r)
            x = - a / 2 - r  # начало отсчета по х
            if j == ky and n < N:
                kx = kx - (N - n)
                dx = (a - kx * (2 * r)) / (kx + 1)  # расстояние между зонами безопасности РТП
            for i in np.arange(1, kx+1):
                # x0 = -(a - k*(2*r))/(k+1)/2 - r
                x = x + dx + np.dot(2, r)
                Virtual_formation[k, :] = np.array([x, y])
                k = k + 1  # абсциса РТП
    elif formation.type == FormationParam.SQUARE:  # Каре - квадрат
        P = n * (formation.distance + 2 * safety_radius_rtp)
        b = P / 4
        if n == 1:
            a = 0
        elif n == 2:
            a = b
        else:
            a = b / 2
        # sizeBuild = [2*a 2*a];
        # -->
        visionAngle = np.deg2rad(360)
        Virtual_formation = np.zeros((n, 2))
        alpha_0 = np.deg2rad(0)
        d_alpha = visionAngle / n
        for i in np.arange(0, n):
            if np.deg2rad(0) <= alpha_0 < np.deg2rad(45):
                Virtual_formation[i, :] = np.array([a, a * np.tan(alpha_0)])
            elif np.deg2rad(45) <= alpha_0 < np.deg2rad(135):
                Virtual_formation[i, :] = np.array([a / np.tan(alpha_0), a])
            elif np.deg2rad(135) <= alpha_0 < np.deg2rad(225):
                Virtual_formation[i, :] = np.array([- a, - a * np.tan(alpha_0)])
            elif np.deg2rad(225) <= alpha_0 < np.deg2rad(315):
                Virtual_formation[i, :] = np.array([- a / np.tan(alpha_0), - a])
            else:
                Virtual_formation[i, :] = np.array([a, a * np.tan(alpha_0)])
            alpha_0 = np.double(alpha_0 + d_alpha)
            if alpha_0 > 2 * np.pi:
                alpha_0 = alpha_0 - 2 * np.pi
    elif formation.type == FormationParam.KLIN:
        d = formation.distance + 2 * safety_radius_rtp
        a = d
        b = d * np.sqrt(3) / 2
        sizeBuild = np.array([np.fix(n / 2) * a, np.fix(n / 2) * b])
        k = 0
        if np.mod(n, 2) == 1:
            Virtual_formation[0, :] = np.array([0, 0])
            k = k + 1
        for i in np.arange(1, np.fix(n / 2) + 1):
            Virtual_formation[k, :] = np.array([(- i) * a / 2, - i * b])
            k = k + 1
            Virtual_formation[k, :] = np.array([i * a / 2, (- i) * b])
            k = k + 1
        # смещение клина вверх
        # -->
        Virtual_formation[:, 1] = Virtual_formation[:, 1] + sizeBuild[1] / 2
        # центр клина (при четном количестве РТП центр будет смещен ещё на 1/4)
        if np.mod(n, 2) == 0:
            # -->
            Virtual_formation[:, 1] = Virtual_formation[:, 1] + b / 2
    elif formation.type == FormationParam.REVERSE_KLIN:
        # -->
        d = formation.distance + 2 * safety_radius_rtp
        a = d
        b = d * np.sqrt(3) / 2
        sizeBuild = np.array([np.fix(n / 2) * a, np.fix(n / 2) * b])
        k = 0
        if np.mod(n, 2) == 1:
            Virtual_formation[0, :] = np.array([0, 0])
            k = k + 1
        for i in np.arange(1, np.fix(n / 2) + 1):
            Virtual_formation[k, :] = np.array([- i * a / 2, i * b])
            k = k + 1
            Virtual_formation[k, :] = np.array([i * a / 2, i * b])
            k = k + 1
        # смещение клина вниз
        # -->
        Virtual_formation[:, 1] = Virtual_formation[:, 1] - sizeBuild[1] / 2
        # центр обратного клина(при четном количестве РТП центр будет смещен
        # ещё на 1/4
        if np.mod(n, 2) == 0:
            # -->
            Virtual_formation[:, 1] = Virtual_formation[:, 1] - b / 2
    elif formation.type == FormationParam.COLUMN:
        # -->
        d = formation.distance + 2 * safety_radius_rtp
        # b = double((n-1)*d + 2*safety_radius_rtp);
        # sizeBuild = [a  b];
        # -->
        Virtual_formation[0, :] = np.array([0, 0])
        for i in np.arange(1, n + 1):
            Virtual_formation[i - 1, :] = np.array([0, - (i - 1) * d])
        y0 = - (n - 1) * d / 2
        Virtual_formation[:, 1] = Virtual_formation[:, 1] - y0
    elif formation.type == FormationParam.ECHELON:
        d = formation.distance + 2 * safety_radius_rtp
        a = d
        # sizeBuild = [(n-1)*a+2*widthRTP  b];
        k = 0
        if np.mod(n, 2) == 1:
            Virtual_formation[0, :] = np.array([0, 0])
            k = k + 1
            for i in np.arange(1, np.fix(n / 2) + 1):
                Virtual_formation[k, :] = np.array([np.dot(- i, a), 0])
                k = k + 1
                Virtual_formation[k, :] = np.array([np.dot(i, a), 0])
                k = k + 1
        else:
            for i in np.arange(0, np.fix(n / 2)):
                Virtual_formation[k, :] = np.array([- a / 2 - i * a, 0])
                k = k + 1
                Virtual_formation[k, :] = np.array([a / 2 + i * a, 0])
                k = k + 1
    elif formation.type == FormationParam.CIRCLE:
        d = formation.distance + 2 * safety_radius_rtp
        # определяется по дуге окружности
        if n == 1:
            a = 0
        elif n == 2:
            a = d / 2
        else:
            a = d * n / (2 * np.pi)
        # sizeBuild = [2*a  2*a]
        alpha_0 = np.deg2rad(0)
        scanAngleResolution = np.deg2rad(np.double(360) / n)
        visionAngle = np.deg2rad(360)
        Virtual_formation = np.zeros((n, 2))
        k = 0
        for i in np.arange(alpha_0, visionAngle + alpha_0, scanAngleResolution):
            Virtual_formation[k, :] = np.array([a * np.cos(i), a * np.sin(i)])
            k = k + 1
    # Коррекция размеров виртуальной структуры
    sX = np.max(Virtual_formation[:, 0]) - np.min(Virtual_formation[:, 0])
    sY = np.max(Virtual_formation[:, 1]) - np.min(Virtual_formation[:, 1])
    if sX < 0.001:
        sX = 2 * safety_radius_rtp
    if sY < 0.001:
        sY = 2 * safety_radius_rtp

    sizeBuild = np.array([sX, sY])

    # ----------------------------------------------------
    # поворот виртуальной структуры на угол alpha
    # -----------------------------------------------------
    if alpha != 0.0:
        f0 = np.zeros((1, n))
        r = np.zeros((1, n))
        for i in np.arange(0, n):
            r[:, i] = np.sqrt(Virtual_formation[i, 0] ** 2 + Virtual_formation[i, 1] ** 2)
            f0[:, i] = myAngle_new(Virtual_formation[i, 0], Virtual_formation[i, 1])

        Virtual_formation[:, 0] = np.multiply(r.flatten(), np.cos(f0.flatten() + alpha))
        Virtual_formation[:, 1] = np.multiply(r.flatten(), np.sin(f0.flatten() + alpha))

    return Virtual_formation, sizeBuild


def myAngle_new(dx, dy):
    """
    myANGLE вычисление угла
    Функция вычисления угла в радианах
    Исходными данными являются расстояния
    между точками по оси ОХ и ОУ соответственно
    :param dx:
    :param dy:
    :return:
    """

    Angle = 0
    if dx > 0 and dy >= 0:
        Angle = np.arctan((dy / dx))
    elif dx > 0 > dy:
        Angle = np.arctan(dy / dx) + 2 * np.pi
    elif dx < 0:
        Angle = np.arctan(dy / dx) + np.pi
    elif dx == 0 and dy > 0:
        Angle = np.pi / 2
    elif dx == 0 and dy < 0:
        Angle = 3 * np.pi / 2
    elif dx == 0 and dy == 0:
        Angle = 0

    return Angle

