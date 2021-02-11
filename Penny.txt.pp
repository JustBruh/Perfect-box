Мобильный колесный робот по типу самосвал:
4 колеса, два из них - ведущие
Момент мотора = 0.15 Н\м
Масса платформы = 2 кг
Частота оборотов мотора = 100 об\мин
Диаметр колес = 50мм
Мощность мотора = ?
Способ управления - регулировка скорости вращение определенного колеса
Контроллер на blutetooth отдает сигнал по bluetooth на RaspberryPi, RaspberryPi отдает команду через GPIO коннекторы на моторы ведущих колес.
Питание платформы, включающей RaspberryPi, моторы - аккумулятор, установленный на самой платформе.
Программирование:
Язык - python
Библиотека для работы с GPIO коннекторами - https://pypi.org/project/RPi.GPIO/
Основные задачи - изменение скорости вращения моторов и их синхронизация