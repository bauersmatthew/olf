"""Holds Error classes that are useful throughout the app."""

class Error(Exception):
    """Base class for our exceptions."""
    pass

class ParseError(Error):
    """Thrown when a string can't be parsed into a valid value."""
    pass

class ConfigError(Error):
    """Thrown when the experiment configuration is invalid."""
    pass

class VersionError(Error):
    """Indicates that an incompatible version was requested."""
    def __init__(self, good, bad, what):
        super().__init__(
            f'Bad {what} protocol version: {bad} requested, {good} provided')
        self.good = good
        self.bad = bad
        self.what = what

class ControllerError(Error):
    """Thrown when there is an issue communicating to the controller."""
    pass