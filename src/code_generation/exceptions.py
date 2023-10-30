
class PepperTestingException(Exception):
    pass

class InvalidObjectException(PepperTestingException):
    pass

class LanguageNotSupported(PepperTestingException):
    pass

class QuestionAnswerTagException(PepperTestingException):
    pass

class InvalidLocationException(PepperTestingException):
    pass

class InvalidDegreesException(PepperTestingException):
    pass

class EmptyQuestionException(PepperTestingException):
    pass