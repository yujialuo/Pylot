
class Stream(object):

    def __init__(self, name, data_type):
        self.name = name
        self.data_type = data_type
        self.sender = None
        self.callback = None
        self.callback_index = None

    def send_to(self, callback, index=None):
        """
        Send execution function
        """
        self.callback = callback
        self.callback_index = index

    def init_sender(self):
        """
        Different type of packet has different implementation
        """
        pass

    def init_receiver(self):
        """
        Different type of packet has different implementation
        """
        pass

    def add(self, data):
        """
        Different type of packet has different implementation
        """
        pass

    def info(self):
        """
        Print out packet info
        """
        print("[Stream] Name: {}, Type: {}".format(self.name, self.data_type))
