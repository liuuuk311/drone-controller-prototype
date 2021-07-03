class ConnectionFailedTooManyTimes(Exception):
    """ Raised when is not possible to connect to the drone for too many times """

    def __init__(self, attempts: int = 2, message="Tried to connect {} time(s) without success"):
        self.message = message.format(attempts)
        super().__init__(self.message)