# @generated by generate_proto_mypy_stubs.py.  Do not edit!
import sys
from google.protobuf.descriptor import (
    Descriptor as google___protobuf___descriptor___Descriptor,
    EnumDescriptor as google___protobuf___descriptor___EnumDescriptor,
)

from google.protobuf.internal.containers import (
    RepeatedScalarFieldContainer as google___protobuf___internal___containers___RepeatedScalarFieldContainer,
)

from google.protobuf.message import (
    Message as google___protobuf___message___Message,
)

from typing import (
    Iterable as typing___Iterable,
    List as typing___List,
    Optional as typing___Optional,
    Tuple as typing___Tuple,
    cast as typing___cast,
)

from typing_extensions import (
    Literal as typing_extensions___Literal,
)


builtin___bool = bool
builtin___bytes = bytes
builtin___float = float
builtin___int = int
builtin___str = str


class CompressionTypeProto(builtin___int):
    DESCRIPTOR: google___protobuf___descriptor___EnumDescriptor = ...
    @classmethod
    def Name(cls, number: builtin___int) -> builtin___str: ...
    @classmethod
    def Value(cls, name: builtin___str) -> 'CompressionTypeProto': ...
    @classmethod
    def keys(cls) -> typing___List[builtin___str]: ...
    @classmethod
    def values(cls) -> typing___List['CompressionTypeProto']: ...
    @classmethod
    def items(cls) -> typing___List[typing___Tuple[builtin___str, 'CompressionTypeProto']]: ...
    NONE = typing___cast('CompressionTypeProto', 0)
    PNG = typing___cast('CompressionTypeProto', 1)
NONE = typing___cast('CompressionTypeProto', 0)
PNG = typing___cast('CompressionTypeProto', 1)

class ObservationProto(google___protobuf___message___Message):
    DESCRIPTOR: google___protobuf___descriptor___Descriptor = ...
    class FloatData(google___protobuf___message___Message):
        DESCRIPTOR: google___protobuf___descriptor___Descriptor = ...
        data = ... # type: google___protobuf___internal___containers___RepeatedScalarFieldContainer[builtin___float]

        def __init__(self,
            *,
            data : typing___Optional[typing___Iterable[builtin___float]] = None,
            ) -> None: ...
        @classmethod
        def FromString(cls, s: builtin___bytes) -> ObservationProto.FloatData: ...
        def MergeFrom(self, other_msg: google___protobuf___message___Message) -> None: ...
        def CopyFrom(self, other_msg: google___protobuf___message___Message) -> None: ...
        if sys.version_info >= (3,):
            def ClearField(self, field_name: typing_extensions___Literal[u"data"]) -> None: ...
        else:
            def ClearField(self, field_name: typing_extensions___Literal[u"data",b"data"]) -> None: ...

    shape = ... # type: google___protobuf___internal___containers___RepeatedScalarFieldContainer[builtin___int]
    compression_type = ... # type: CompressionTypeProto
    compressed_data = ... # type: builtin___bytes
    compressed_channel_mapping = ... # type: google___protobuf___internal___containers___RepeatedScalarFieldContainer[builtin___int]

    @property
    def float_data(self) -> ObservationProto.FloatData: ...

    def __init__(self,
        *,
        shape : typing___Optional[typing___Iterable[builtin___int]] = None,
        compression_type : typing___Optional[CompressionTypeProto] = None,
        compressed_data : typing___Optional[builtin___bytes] = None,
        float_data : typing___Optional[ObservationProto.FloatData] = None,
        compressed_channel_mapping : typing___Optional[typing___Iterable[builtin___int]] = None,
        ) -> None: ...
    @classmethod
    def FromString(cls, s: builtin___bytes) -> ObservationProto: ...
    def MergeFrom(self, other_msg: google___protobuf___message___Message) -> None: ...
    def CopyFrom(self, other_msg: google___protobuf___message___Message) -> None: ...
    if sys.version_info >= (3,):
        def HasField(self, field_name: typing_extensions___Literal[u"compressed_data",u"float_data",u"observation_data"]) -> builtin___bool: ...
        def ClearField(self, field_name: typing_extensions___Literal[u"compressed_channel_mapping",u"compressed_data",u"compression_type",u"float_data",u"observation_data",u"shape"]) -> None: ...
    else:
        def HasField(self, field_name: typing_extensions___Literal[u"compressed_data",b"compressed_data",u"float_data",b"float_data",u"observation_data",b"observation_data"]) -> builtin___bool: ...
        def ClearField(self, field_name: typing_extensions___Literal[u"compressed_channel_mapping",b"compressed_channel_mapping",u"compressed_data",b"compressed_data",u"compression_type",b"compression_type",u"float_data",b"float_data",u"observation_data",b"observation_data",u"shape",b"shape"]) -> None: ...
    def WhichOneof(self, oneof_group: typing_extensions___Literal[u"observation_data",b"observation_data"]) -> typing_extensions___Literal["compressed_data","float_data"]: ...