// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: MessageType.proto

#include "MessageType.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace Protobuf {
}  // namespace Protobuf
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_MessageType_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_MessageType_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_MessageType_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_MessageType_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_MessageType_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\021MessageType.proto\022\010Protobuf*\246\001\n\013Messag"
  "eType\022\r\n\tAddPlayer\020\000\022\010\n\004Move\020\001\022\n\n\006Attack"
  "\020\002\022\010\n\004Pick\020\003\022\007\n\003Use\020\004\022\t\n\005Throw\020\005\022\010\n\004Send"
  "\020\006\022\021\n\rInvalidPlayer\020\007\022\017\n\013ValidPlayer\020\010\022\r"
  "\n\tStartGame\020\t\022\n\n\006Gaming\020\n\022\013\n\007EndGame\020\013B\026"
  "\252\002\023Communication.Protob\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_MessageType_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_MessageType_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_MessageType_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_MessageType_2eproto = {
  false, false, descriptor_table_protodef_MessageType_2eproto, "MessageType.proto", 230,
  &descriptor_table_MessageType_2eproto_once, descriptor_table_MessageType_2eproto_sccs, descriptor_table_MessageType_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_MessageType_2eproto::offsets,
  file_level_metadata_MessageType_2eproto, 0, file_level_enum_descriptors_MessageType_2eproto, file_level_service_descriptors_MessageType_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_MessageType_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_MessageType_2eproto)), true);
namespace Protobuf {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MessageType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_MessageType_2eproto);
  return file_level_enum_descriptors_MessageType_2eproto[0];
}
bool MessageType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace Protobuf
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
