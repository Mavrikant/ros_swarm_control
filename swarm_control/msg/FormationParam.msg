byte NO_DATA=0  	# нет данных
byte RANK=1 		# Ширенга
byte SQUARE=2		# Квадрат
byte KLIN=3		# Клин
byte REVERSE_KLIN=4	# Обратный клин
byte COLUMN=5		# Колонна
byte ECHELON=6		# Эшелон
byte CIRCLE=7		# Круг

byte type		# type of formation

int8 leader_id		# id лидера, если -1 лидер не задан

float32 distance
float32 distance_error
float32[] virtual_formation
float32[] size_formation
