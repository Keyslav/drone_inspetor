"""Testes para drone_inspetor.common.msg_utils."""

import pytest

from drone_inspetor.common.msg_utils import msg_to_dict


class FakeMsgSimple:
    """Simula uma mensagem ROS2 com get_fields_and_field_types."""

    def __init__(self):
        self.state = 10
        self.is_armed = True
        self.altitude = 12.5
        self.name = "test"

    @staticmethod
    def get_fields_and_field_types():
        return {
            "state": "int32",
            "is_armed": "boolean",
            "altitude": "double",
            "name": "string",
        }


class FakeMsgWithSequence:
    """Simula mensagem com campo sequence."""

    def __init__(self):
        self.values = (1.0, 2.0, 3.0)
        self.tags = ["a", "b"]
        self.count = 3

    @staticmethod
    def get_fields_and_field_types():
        return {
            "values": "sequence<double>",
            "tags": "sequence<string>",
            "count": "int32",
        }


class TestMsgToDict:
    def test_converts_all_fields(self):
        msg = FakeMsgSimple()
        result = msg_to_dict(msg)
        assert result == {
            "state": 10,
            "is_armed": True,
            "altitude": 12.5,
            "name": "test",
        }

    def test_returns_dict_type(self):
        result = msg_to_dict(FakeMsgSimple())
        assert isinstance(result, dict)

    def test_sequence_converted_to_list(self):
        msg = FakeMsgWithSequence()
        result = msg_to_dict(msg)
        assert isinstance(result["values"], list)
        assert result["values"] == [1.0, 2.0, 3.0]
        assert isinstance(result["tags"], list)
        assert result["tags"] == ["a", "b"]
        assert result["count"] == 3  # escalar permanece escalar

    def test_empty_message(self):
        class EmptyMsg:
            @staticmethod
            def get_fields_and_field_types():
                return {}
        assert msg_to_dict(EmptyMsg()) == {}

    def test_field_count_matches(self):
        msg = FakeMsgSimple()
        assert len(msg_to_dict(msg)) == len(msg.get_fields_and_field_types())
