// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Message2Server.proto

#include "Message2Server.pb.h"

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
class MessageToServerDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<MessageToServer> _instance;
} _MessageToServer_default_instance_;
}  // namespace Protobuf
static void InitDefaultsscc_info_MessageToServer_Message2Server_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::Protobuf::_MessageToServer_default_instance_;
    new (ptr) ::Protobuf::MessageToServer();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_MessageToServer_Message2Server_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_MessageToServer_Message2Server_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_Message2Server_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_Message2Server_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_Message2Server_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_Message2Server_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, messagetype_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, playerid_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, teamid_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, jobtype_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, proptype_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, timeinmilliseconds_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, angle_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, toplayerid_),
  PROTOBUF_FIELD_OFFSET(::Protobuf::MessageToServer, message_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::Protobuf::MessageToServer)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::Protobuf::_MessageToServer_default_instance_),
};

const char descriptor_table_protodef_Message2Server_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\024Message2Server.proto\022\010Protobuf\032\021Messag"
  "eType.proto\032\024Message2Client.proto\"\371\001\n\017Me"
  "ssageToServer\022*\n\013messageType\030\001 \001(\0162\025.Pro"
  "tobuf.MessageType\022\020\n\010playerID\030\002 \001(\003\022\016\n\006t"
  "eamID\030\003 \001(\003\022\"\n\007jobType\030\004 \001(\0162\021.Protobuf."
  "JobType\022$\n\010propType\030\005 \001(\0162\022.Protobuf.Pro"
  "pType\022\032\n\022timeInMilliseconds\030\006 \001(\005\022\r\n\005ang"
  "le\030\007 \001(\001\022\022\n\nToPlayerID\030\010 \001(\003\022\017\n\007message\030"
  "\t \001(\tB\026\252\002\023Communication.Protob\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_Message2Server_2eproto_deps[2] = {
  &::descriptor_table_Message2Client_2eproto,
  &::descriptor_table_MessageType_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_Message2Server_2eproto_sccs[1] = {
  &scc_info_MessageToServer_Message2Server_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_Message2Server_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_Message2Server_2eproto = {
  false, false, descriptor_table_protodef_Message2Server_2eproto, "Message2Server.proto", 357,
  &descriptor_table_Message2Server_2eproto_once, descriptor_table_Message2Server_2eproto_sccs, descriptor_table_Message2Server_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_Message2Server_2eproto::offsets,
  file_level_metadata_Message2Server_2eproto, 1, file_level_enum_descriptors_Message2Server_2eproto, file_level_service_descriptors_Message2Server_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_Message2Server_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_Message2Server_2eproto)), true);
namespace Protobuf {

// ===================================================================

class MessageToServer::_Internal {
 public:
};

MessageToServer::MessageToServer(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:Protobuf.MessageToServer)
}
MessageToServer::MessageToServer(const MessageToServer& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  message_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (!from._internal_message().empty()) {
    message_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_message(), 
      GetArena());
  }
  ::memcpy(&playerid_, &from.playerid_,
    static_cast<size_t>(reinterpret_cast<char*>(&toplayerid_) -
    reinterpret_cast<char*>(&playerid_)) + sizeof(toplayerid_));
  // @@protoc_insertion_point(copy_constructor:Protobuf.MessageToServer)
}

void MessageToServer::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_MessageToServer_Message2Server_2eproto.base);
  message_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
      reinterpret_cast<char*>(&playerid_) - reinterpret_cast<char*>(this)),
      0, static_cast<size_t>(reinterpret_cast<char*>(&toplayerid_) -
      reinterpret_cast<char*>(&playerid_)) + sizeof(toplayerid_));
}

MessageToServer::~MessageToServer() {
  // @@protoc_insertion_point(destructor:Protobuf.MessageToServer)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void MessageToServer::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  message_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void MessageToServer::ArenaDtor(void* object) {
  MessageToServer* _this = reinterpret_cast< MessageToServer* >(object);
  (void)_this;
}
void MessageToServer::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void MessageToServer::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const MessageToServer& MessageToServer::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_MessageToServer_Message2Server_2eproto.base);
  return *internal_default_instance();
}


void MessageToServer::Clear() {
// @@protoc_insertion_point(message_clear_start:Protobuf.MessageToServer)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  message_.ClearToEmpty();
  ::memset(&playerid_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&toplayerid_) -
      reinterpret_cast<char*>(&playerid_)) + sizeof(toplayerid_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MessageToServer::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .Protobuf.MessageType messageType = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_messagetype(static_cast<::Protobuf::MessageType>(val));
        } else goto handle_unusual;
        continue;
      // int64 playerID = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          playerid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // int64 teamID = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          teamid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .Protobuf.JobType jobType = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_jobtype(static_cast<::Protobuf::JobType>(val));
        } else goto handle_unusual;
        continue;
      // .Protobuf.PropType propType = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_proptype(static_cast<::Protobuf::PropType>(val));
        } else goto handle_unusual;
        continue;
      // int32 timeInMilliseconds = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          timeinmilliseconds_ = static_cast<uint32_t>(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // double angle = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 57)) {
          angle_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // int64 ToPlayerID = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          toplayerid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // string message = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          auto str = _internal_mutable_message();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "Protobuf.MessageToServer.message"));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* MessageToServer::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Protobuf.MessageToServer)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .Protobuf.MessageType messageType = 1;
  if (this->messagetype() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_messagetype(), target);
  }

  // int64 playerID = 2;
  if (this->playerid() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(2, this->_internal_playerid(), target);
  }

  // int64 teamID = 3;
  if (this->teamid() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(3, this->_internal_teamid(), target);
  }

  // .Protobuf.JobType jobType = 4;
  if (this->jobtype() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      4, this->_internal_jobtype(), target);
  }

  // .Protobuf.PropType propType = 5;
  if (this->proptype() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      5, this->_internal_proptype(), target);
  }

  // int32 timeInMilliseconds = 6;
  if (this->timeinmilliseconds() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(6, this->_internal_timeinmilliseconds(), target);
  }

  // double angle = 7;
  if (!(this->angle() <= 0 && this->angle() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(7, this->_internal_angle(), target);
  }

  // int64 ToPlayerID = 8;
  if (this->toplayerid() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(8, this->_internal_toplayerid(), target);
  }

  // string message = 9;
  if (this->message().size() > 0) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_message().data(), static_cast<int>(this->_internal_message().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "Protobuf.MessageToServer.message");
    target = stream->WriteStringMaybeAliased(
        9, this->_internal_message(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Protobuf.MessageToServer)
  return target;
}

size_t MessageToServer::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Protobuf.MessageToServer)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string message = 9;
  if (this->message().size() > 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_message());
  }

  // int64 playerID = 2;
  if (this->playerid() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
        this->_internal_playerid());
  }

  // .Protobuf.MessageType messageType = 1;
  if (this->messagetype() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_messagetype());
  }

  // .Protobuf.JobType jobType = 4;
  if (this->jobtype() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_jobtype());
  }

  // int64 teamID = 3;
  if (this->teamid() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
        this->_internal_teamid());
  }

  // .Protobuf.PropType propType = 5;
  if (this->proptype() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_proptype());
  }

  // int32 timeInMilliseconds = 6;
  if (this->timeinmilliseconds() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_timeinmilliseconds());
  }

  // double angle = 7;
  if (!(this->angle() <= 0 && this->angle() >= 0)) {
    total_size += 1 + 8;
  }

  // int64 ToPlayerID = 8;
  if (this->toplayerid() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
        this->_internal_toplayerid());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void MessageToServer::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Protobuf.MessageToServer)
  GOOGLE_DCHECK_NE(&from, this);
  const MessageToServer* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<MessageToServer>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Protobuf.MessageToServer)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Protobuf.MessageToServer)
    MergeFrom(*source);
  }
}

void MessageToServer::MergeFrom(const MessageToServer& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Protobuf.MessageToServer)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.message().size() > 0) {
    _internal_set_message(from._internal_message());
  }
  if (from.playerid() != 0) {
    _internal_set_playerid(from._internal_playerid());
  }
  if (from.messagetype() != 0) {
    _internal_set_messagetype(from._internal_messagetype());
  }
  if (from.jobtype() != 0) {
    _internal_set_jobtype(from._internal_jobtype());
  }
  if (from.teamid() != 0) {
    _internal_set_teamid(from._internal_teamid());
  }
  if (from.proptype() != 0) {
    _internal_set_proptype(from._internal_proptype());
  }
  if (from.timeinmilliseconds() != 0) {
    _internal_set_timeinmilliseconds(from._internal_timeinmilliseconds());
  }
  if (!(from.angle() <= 0 && from.angle() >= 0)) {
    _internal_set_angle(from._internal_angle());
  }
  if (from.toplayerid() != 0) {
    _internal_set_toplayerid(from._internal_toplayerid());
  }
}

void MessageToServer::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Protobuf.MessageToServer)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MessageToServer::CopyFrom(const MessageToServer& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Protobuf.MessageToServer)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MessageToServer::IsInitialized() const {
  return true;
}

void MessageToServer::InternalSwap(MessageToServer* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  message_.Swap(&other->message_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(MessageToServer, toplayerid_)
      + sizeof(MessageToServer::toplayerid_)
      - PROTOBUF_FIELD_OFFSET(MessageToServer, playerid_)>(
          reinterpret_cast<char*>(&playerid_),
          reinterpret_cast<char*>(&other->playerid_));
}

::PROTOBUF_NAMESPACE_ID::Metadata MessageToServer::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace Protobuf
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::Protobuf::MessageToServer* Arena::CreateMaybeMessage< ::Protobuf::MessageToServer >(Arena* arena) {
  return Arena::CreateMessageInternal< ::Protobuf::MessageToServer >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
