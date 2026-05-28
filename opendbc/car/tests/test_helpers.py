import unittest
from opendbc.car import make_communication_control_msg, CanData


class TestCommunicationControlMsg(unittest.TestCase):
  def test_enable_rx_tx_with_suppress_response(self):
    msg = make_communication_control_msg(0x730, 0, sub_function=0x00, suppress_response=True)
    self.assertIsInstance(msg, CanData)
    self.assertEqual(msg.address, 0x730)
    self.assertEqual(msg.src, 0)
    # ISO 14229-1: byte 0 = length (3), byte 1 = 0x28 (CommunicationControl),
    # byte 2 = subFunction | 0x80 (suppress), byte 3 = communicationType (0x01).
    self.assertEqual(msg.dat[0], 0x03)
    self.assertEqual(msg.dat[1], 0x28)
    self.assertEqual(msg.dat[2], 0x80)  # subFunction 0x00 | suppress 0x80
    self.assertEqual(msg.dat[3], 0x01)

  def test_disable_rx_tx_no_suppress(self):
    msg = make_communication_control_msg(0x730, 0, sub_function=0x03, suppress_response=False)
    self.assertEqual(msg.dat[2], 0x03)


if __name__ == "__main__":
  unittest.main()
