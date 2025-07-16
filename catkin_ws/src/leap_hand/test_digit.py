from digit_interface import Digit
from digit_interface import DigitHandler

digits = DigitHandler.list_digits()
print(f"digits: {digits}") # returns digits: [{'dev_name': '/dev/video2', 'manufacturer': 'Facebook', 'model': 'DIGIT', 'revision': '0200', 'serial': 'D20262'}, {'dev_name': '/dev/video3', 'manufacturer': 'Facebook', 'model': 'DIGIT', 'revision': '0200', 'serial': 'D20262'}]


d = Digit("D20262") # Unique serial number
d.connect()
d.show_view()
d.disconenect()
# frame = d.get_frame()
