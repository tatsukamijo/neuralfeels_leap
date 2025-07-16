from digit_interface import Digit
from digit_interface import DigitHandler

digits = DigitHandler.list_digits()
print(f"digits: {digits}")

d = Digit("D20262") # Unique serial number
d.connect()
d.show_view()
d.disconenect()
# frame = d.get_frame()
