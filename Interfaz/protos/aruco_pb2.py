# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: aruco.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0b\x61ruco.proto\x12\x05\x61ruco\" \n\rMarkerCorners\x12\x0f\n\x07\x63orners\x18\x01 \x03(\x02\"\x1e\n\x0cMarkerCenter\x12\x0e\n\x06\x63\x65nter\x18\x01 \x03(\x02\"i\n\x0f\x41rucoMarkerData\x12\n\n\x02id\x18\x01 \x01(\x05\x12%\n\x07\x63orners\x18\x02 \x01(\x0b\x32\x14.aruco.MarkerCorners\x12#\n\x06\x63\x65nter\x18\x03 \x01(\x0b\x32\x13.aruco.MarkerCenter\"<\n\x0bImageResult\x12\x0e\n\x06\x62\x36\x34img\x18\x01 \x01(\t\x12\r\n\x05width\x18\x02 \x01(\x05\x12\x0e\n\x06height\x18\x03 \x01(\x05\"\x07\n\x05\x45mpty2|\n\rArucoDetector\x12\x37\n\rDetectMarkers\x12\x0c.aruco.Empty\x1a\x16.aruco.ArucoMarkerData0\x01\x12\x32\n\x0eGetImageResult\x12\x0c.aruco.Empty\x1a\x12.aruco.ImageResultb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'aruco_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_MARKERCORNERS']._serialized_start=22
  _globals['_MARKERCORNERS']._serialized_end=54
  _globals['_MARKERCENTER']._serialized_start=56
  _globals['_MARKERCENTER']._serialized_end=86
  _globals['_ARUCOMARKERDATA']._serialized_start=88
  _globals['_ARUCOMARKERDATA']._serialized_end=193
  _globals['_IMAGERESULT']._serialized_start=195
  _globals['_IMAGERESULT']._serialized_end=255
  _globals['_EMPTY']._serialized_start=257
  _globals['_EMPTY']._serialized_end=264
  _globals['_ARUCODETECTOR']._serialized_start=266
  _globals['_ARUCODETECTOR']._serialized_end=390
# @@protoc_insertion_point(module_scope)
