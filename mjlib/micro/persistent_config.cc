// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified by Tenax Motion, 2026: the number of registered names is a
// constructor option, and further command managers can each be given
// a view of the groups with one prefix (AddView).

#include "mjlib/micro/persistent_config.h"

#include <cstring>

#include <boost/crc.hpp>

#include "mjlib/base/assert.h"
#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/crc.h"
#include "mjlib/base/crc_stream.h"
#include "mjlib/base/null_stream.h"
#include "mjlib/base/tokenizer.h"

#include "mjlib/telemetry/format.h"

#include "mjlib/micro/flash.h"
#include "mjlib/micro/pool_map.h"

/// @file
///
/// The persistent storage is an unordered list of elements:
///
///   * Element
///     * pstring - name
///     * 32bit schema CRC
///     * pstring - data
///
/// It is terminated by an element with a 0 length name.
///
/// A pstring is a 32 byte unsigned integer followed by that many
/// bytes of data.

namespace mjlib {
namespace micro {
namespace {
class SizeCountingStream : public base::WriteStream {
 public:
  void write(const std::string_view& data) override {
    size_ += data.size();
  }

  size_t size() const { return size_; }

 private:
  size_t size_ = 0;
};
}

class PersistentConfig::Impl {
 public:
  Impl(Pool& pool, CommandManager& command_manager, FlashInterface& flash,
       mjlib::base::string_span output_buffer, size_t max_elements)
      : pool_(pool), flash_(flash), elements_(&pool, max_elements) {
    primary_.output_buffer = output_buffer;
    command_manager.Register("conf", [this](auto&& a, auto&& b) {
        this->Command(primary_, a, b);
      });
  }

  /// What one command manager is served with: the groups it is shown,
  /// and the state of the command it has in progress.
  struct View {
    /// Empty for the view of the constructor's command manager.
    std::string_view prefix;
    mjlib::base::string_span output_buffer;
    std::size_t current_list_index = 0;

    CommandManager::Response current_response;
    std::size_t current_enumerate_index = 0;
    std::size_t end_enumerate_index = 0;
    bool enumerate_one = false;
    detail::EnumerateArchive::Context enumerate_context;

    View* next = nullptr;
  };

  void AddView(CommandManager& command_manager,
               const std::string_view& prefix,
               mjlib::base::string_span output_buffer) {
    MJ_ASSERT(!prefix.empty());

    PoolPtr<View> view(&pool_);
    view->prefix = prefix;
    view->output_buffer = output_buffer;

    View* last = &primary_;
    while (last->next) { last = last->next; }
    last->next = view.get();

    command_manager.Register(
        "conf", [this, v = view.get()](auto&& a, auto&& b) {
          this->Command(*v, a, b);
        });
  }

  /// The view a group belongs to: the one with the longest prefix that
  /// the name of the group starts with.
  const View* Owner(const std::string_view& name) const {
    const View* result = &primary_;
    for (const View* view = primary_.next; view; view = view->next) {
      if (view->prefix.size() > result->prefix.size() &&
          name.substr(0, view->prefix.size()) == view->prefix) {
        result = view;
      }
    }
    return result;
  }

  /// If @p view has a group of its own which it calls @p name.
  bool HasOwn(const View& view, const std::string_view& name) const {
    for (const auto& item : elements_) {
      if (IsOwnName(view, item.first, name)) { return true; }
    }
    return false;
  }

  static bool IsOwnName(const View& view,
                        const std::string_view& full_name,
                        const std::string_view& name) {
    return full_name.size() == view.prefix.size() + name.size() &&
        full_name.substr(0, view.prefix.size()) == view.prefix &&
        full_name.substr(view.prefix.size()) == name;
  }

  /// If @p view lists the group: its own, and each group of no view
  /// (one that every view shares) that none of its own has the name
  /// of.
  bool Shows(const View& view, const std::string_view& full_name) const {
    const View* const owner = Owner(full_name);
    if (owner == &view) { return true; }
    return owner == &primary_ && !HasOwn(view, full_name);
  }

  /// What @p view calls the group.
  std::string_view NameIn(const View& view,
                          const std::string_view& full_name) const {
    return Owner(full_name) == &view ?
        full_name.substr(view.prefix.size()) : full_name;
  }

  /// The group @p view calls @p name: its own of that name if it has
  /// one, otherwise the group with exactly that name.
  auto Find(const View& view, const std::string_view& name) {
    if (!view.prefix.empty()) {
      for (auto it = elements_.begin(); it != elements_.end(); ++it) {
        if (IsOwnName(view, it->first, name)) { return it; }
      }
    }
    return elements_.find(name);
  }

  void Command(View& view,
               const std::string_view& command,
               const CommandManager::Response& response) {
    base::Tokenizer tokenizer(command, " ");
    auto cmd = tokenizer.next();
    if (cmd == "enumerate") {
      Enumerate(view, tokenizer.remaining(), response);
    } else if (cmd == "list") {
      List(view, response);
    } else if (cmd == "schema") {
      Schema(view, tokenizer.remaining(), response);
    } else if (cmd == "data") {
      Data(view, tokenizer.remaining(), response);
    } else if (cmd == "get") {
      Get(view, tokenizer.remaining(), response);
    } else if (cmd == "set") {
      Set(view, tokenizer.remaining(), response);
    } else if (cmd == "load") {
      Load(response);
    } else if (cmd == "write") {
      Write(response);
    } else if (cmd == "size") {
      Size(view, response);
    } else if (cmd == "default") {
      Default(view, response);
    } else {
      UnknownCommand(cmd, response);
    }
  }

  struct Element {
    SerializableHandlerBase* serializable = nullptr;
    base::inplace_function<void ()> updated;
    bool enumerate = true;
  };

  using ElementMap = PoolMap<std::string_view, Element>;

  void Enumerate(View& view,
                 const std::string_view& maybe_field,
                 const CommandManager::Response& response) {
    view.current_response = response;
    if (maybe_field.size()) {
      // Look up this field.
      const auto element_it = Find(view, maybe_field);
      if (element_it == elements_.end()) {
        WriteMessage("ERR unknown group\r\n", response);
        return;
      }
      view.current_enumerate_index = element_it - elements_.begin();
      view.end_enumerate_index = view.current_enumerate_index + 1;
      view.enumerate_one = true;
    } else {
      view.current_enumerate_index = 0;
      view.end_enumerate_index = elements_.size();
      view.enumerate_one = false;
    }

    EnumerateCallback(view, {});
  }

  void EnumerateCallback(View& view, error_code error) {
    if (error) {
      view.current_response.callback(error);
      return;
    }

    while (true) {
      const auto element_it =
          elements_.begin() + view.current_enumerate_index;
      if (view.current_enumerate_index >= view.end_enumerate_index ||
          element_it == elements_.end()) {
        WriteOK(view.current_response);
        return;
      }

      view.current_enumerate_index++;

      if (!element_it->second.enumerate) {
        continue;
      }

      // A group asked for by name is given whoever it belongs to.
      if (!view.enumerate_one && !Shows(view, element_it->first)) {
        continue;
      }

      element_it->second.serializable->Enumerate(
          &view.enumerate_context,
          view.output_buffer,
          NameIn(view, element_it->first),
          *view.current_response.stream,
          [this, v = &view](error_code err) {
            this->EnumerateCallback(*v, err);
          });
      return;
    }
  }

  void List(View& view, const CommandManager::Response& response) {
    view.current_response = response;
    view.current_list_index = 0;
    ListCallback(view, {});
  }

  void ListCallback(View& view, error_code error) {
    if (error) {
      view.current_response.callback(error);
      return;
    }

    while (view.current_list_index < elements_.size() &&
           !Shows(view, elements_[view.current_list_index].first)) {
      view.current_list_index++;
    }

    if (view.current_list_index >= elements_.size()) {
      WriteOK(view.current_response);
      return;
    }

    auto element_it = elements_.begin() + view.current_list_index;
    if (element_it == elements_.end()) {
      WriteOK(view.current_response);
      return;
    }

    const auto name = NameIn(view, element_it->first);
    view.current_list_index++;

    char *ptr = view.output_buffer.data();
    std::copy(name.begin(), name.end(), ptr);
    ptr += name.size();
    *ptr = '\r';
    ptr++;
    *ptr = '\n';
    ptr++;
    AsyncWrite(*view.current_response.stream,
               std::string_view(view.output_buffer.data(),
                                ptr - view.output_buffer.data()),
               [this, v = &view](error_code ec) {
                 ListCallback(*v, ec);
               });
  }

  typedef base::inplace_function<void (Element*,
                                       base::WriteStream*)> WorkFunction;
  void Emit(View& view,
            const std::string_view& prefix,
            const std::string_view& name,
            Element* element,
            WorkFunction work,
            const CommandManager::Response& response) {
    // The prefix+name+CRLF write below cannot itself overflow because
    // BufferWriteStream::write asserts. But we must also make sure
    // there's room for the four-byte size placeholder we reserve via
    // skip() and for whatever work() chooses to emit. Bail with an
    // error response if not -- this used to silently corrupt memory
    // past view.output_buffer when prefix+name+CRLF filled the buffer
    // within < 4 bytes of the end and work() wrote zero bytes.
    if (prefix.size() + name.size() + 2 + sizeof(uint32_t) >
        static_cast<size_t>(view.output_buffer.size())) {
      WriteMessage("ERR name too long\r\n", response);
      return;
    }

    base::BufferWriteStream ostream{view.output_buffer};
    ostream.write(prefix);
    ostream.write(name);
    ostream.write("\r\n");

    char* const size_position = view.output_buffer.data() + ostream.offset();
    ostream.skip(sizeof(uint32_t));

    work(element, &ostream);

    base::BufferWriteStream size_stream({size_position, sizeof(uint32_t)});
    mjlib::telemetry::WriteStream tstream(size_stream);
    tstream.Write(static_cast<uint32_t>(
                      ostream.offset() + view.output_buffer.data() -
                      (size_position + sizeof(uint32_t))));

    AsyncWrite(
        *response.stream,
        std::string_view(view.output_buffer.data(), ostream.offset()),
        response.callback);
  }

  void Schema(View& view,
              const std::string_view& name,
              const CommandManager::Response& response) {
    const auto element_it = Find(view, name);
    if (element_it == elements_.end()) {
      WriteMessage("ERR unknown name\r\n", response);
      return;
    }

    Emit(
        view,
        "cschema ",
        name,
        &element_it->second,
        [](Element* element, base::WriteStream* ostream) {
          element->serializable->WriteSchema(*ostream);
        },
        response);
  }

  void Data(View& view,
            const std::string_view& name,
            const CommandManager::Response& response) {
    const auto it = Find(view, name);
    if (it == elements_.end()) {
      WriteMessage(std::string_view("ERR unknown name\r\n"), response);
      return;
    }

    Emit(view,
         "cdata ",
         name,
         &it->second,
         [](Element* element, base::WriteStream* stream) {
           element->serializable->WriteBinary(*stream);
         },
         response);
  }

  void Get(View& view,
           const std::string_view& field,
           const CommandManager::Response& response) {
    base::Tokenizer tokenizer(field, ".");
    auto group = tokenizer.next();
    const auto element_it = Find(view, group);
    if (element_it == elements_.end()) {
      WriteMessage("ERR unknown group\r\n", response);
    } else {
      view.current_response = response;
      auto& element = element_it->second;
      const int err =
          element.serializable->Read(
              tokenizer.remaining(),
              view.output_buffer,
              *view.current_response.stream,
              [this, v = &view](error_code error) {
                if (error) {
                  v->current_response.callback(error);
                  return;
                }
                WriteMessage("\r\n", v->current_response);
              });
      if (err) {
        WriteMessage("ERR error reading\r\n", response);
      }
    }
  }

  void Set(View& view,
           const std::string_view& command,
           const CommandManager::Response& response) {
    base::Tokenizer tokenizer(command, ".");
    auto group = tokenizer.next();
    const auto element_it = Find(view, group);
    if (element_it == elements_.end()) {
      WriteMessage("ERR unknown group\r\n", response);
    } else {
      base::Tokenizer name_value(tokenizer.remaining(), " ");
      auto key = name_value.next();
      auto value = name_value.remaining();
      auto& element = element_it->second;
      const int result = element.serializable->Set(key, value);
      if (result == 0) {
        element.updated();
        WriteOK(response);
      } else {
        WriteMessage("ERR error setting\r\n", response);
      }
    }
  }

  void Load(const CommandManager::Response& response) {
    DoLoad();

    WriteOK(response);
  }

  void DoLoad() {
    auto info = flash_.GetInfo();
    base::BufferReadStream flash_stream(
        std::string_view(info.start, info.end - info.start));
    telemetry::ReadStream stream(flash_stream);

    while (true) {
      const auto maybe_name_size = stream.ReadVaruint();
      if (!maybe_name_size) {
        // Whoops, an error of some sort.
        break;
      }
      const auto name_size = *maybe_name_size;
      typedef telemetry::Format TF;
      if (name_size == 0 ||
          name_size >= static_cast<uint32_t>(TF::kMaxStringSize)) {
        break;
      }
      std::string_view name(flash_stream.position(), name_size);
      flash_stream.ignore(name_size);

      if (flash_stream.remaining() < 8) { break; }

      const auto maybe_expected_crc = stream.Read<uint32_t>();
      const auto maybe_data_size = stream.Read<uint32_t>();

      if (!maybe_expected_crc || !maybe_data_size) { break; }
      const auto expected_crc = *maybe_expected_crc;
      const auto data_size = *maybe_data_size;

      // We are now committed to reading the entirety of the data one
      // way or another.

      const auto element_it = elements_.find(name);
      if (element_it == elements_.end()) {
        // TODO jpieper: It would be nice to warn about situations
        // like this.
        flash_stream.ignore(data_size);
        continue;
      }

      auto& element = element_it->second;

      const uint32_t actual_crc = CalculateSchemaCrc(element.serializable);
      if (actual_crc != expected_crc) {
        // TODO jpieper: It would be nice to warn about situations like
        // this.
        flash_stream.ignore(data_size);
        continue;
      }

      if (element.serializable->ReadBinary(flash_stream)) {
        // It would be nice to let the caller know this failed.
      }
    }

    // Notify everyone that they have changed.
    for (auto& element: elements_) {
      element.second.updated();
    }
  }

  uint32_t CalculateSchemaCrc(SerializableHandlerBase* base) const {
    base::NullWriteStream null;
    base::CrcWriteStream<boost::crc_32_type> crc_stream(null);

    base->WriteSchema(crc_stream);

    return crc_stream.checksum();
  }

  void Write(const CommandManager::Response& response) {
    auto info = flash_.GetInfo();
    flash_.Unlock();
    flash_.Erase();
    FlashWriteStream flash_stream(flash_, info.start);
    telemetry::WriteStream stream(flash_stream);

    for (const auto& item_pair : elements_) {
      const auto& element = item_pair.second;

      stream.WriteString(item_pair.first);
      stream.Write(static_cast<uint32_t>(
                       CalculateSchemaCrc(element.serializable)));

      SizeCountingStream size_stream;
      element.serializable->WriteBinary(size_stream);
      const uint32_t data_size = size_stream.size();

      stream.Write(data_size);
      element.serializable->WriteBinary(flash_stream);
    }

    stream.Write(static_cast<uint32_t>(0));

    flash_.Lock();

    WriteOK(response);
  }

  void Size(View& view, const CommandManager::Response& response) {
    SizeCountingStream size_stream;
    telemetry::WriteStream stream(size_stream);

    for (const auto& item_pair : elements_) {
      const auto& element = item_pair.second;
      stream.WriteString(item_pair.first);
      stream.Write(static_cast<uint32_t>(0));  // crc
      stream.Write(static_cast<uint32_t>(0)); // size
      element.serializable->WriteBinary(size_stream);
    }
    stream.Write(static_cast<uint32_t>(0));

    ::snprintf(view.output_buffer.data(), view.output_buffer.size(),
               "%d\r\nOK\r\n", static_cast<int>(size_stream.size()));
    WriteMessage(view.output_buffer.data(), response);
  }

  void Default(View& view, const CommandManager::Response& response) {
    // Each view restores its own groups, and the first also those
    // that every view shares.
    for (auto& item_pair : elements_) {
      if (Owner(item_pair.first) != &view) { continue; }
      item_pair.second.serializable->SetDefault();
    }

    // Notify everyone that they have changed.
    for (auto& element: elements_) {
      if (Owner(element.first) != &view) { continue; }
      element.second.updated();
    }

    WriteOK(response);
  }

  void WriteOK(const CommandManager::Response& response) {
    WriteMessage("OK\r\n", response);
  }

  void UnknownCommand(const std::string_view&,
                      const CommandManager::Response& response) {
    WriteMessage("ERR unknown subcommand\r\n", response);
  }

  void WriteMessage(const std::string_view& message,
                    const CommandManager::Response& response) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  Pool& pool_;
  FlashInterface& flash_;

  ElementMap elements_;

  View primary_;
};

PersistentConfig::PersistentConfig(
    Pool& pool, CommandManager& command_manager, FlashInterface& flash,
    mjlib::base::string_span output_buffer, size_t max_elements)
    : impl_(&pool, pool, command_manager, flash, output_buffer,
            max_elements) {
}

PersistentConfig::~PersistentConfig() {
}

void PersistentConfig::AddView(
    CommandManager& command_manager,
    const std::string_view& prefix,
    mjlib::base::string_span output_buffer) {
  impl_->AddView(command_manager, prefix, output_buffer);
}

void PersistentConfig::Load() {
  impl_->DoLoad();
}

void PersistentConfig::RegisterDetail(
    const std::string_view& name,
    void* serializable,
    const TypeErasedSerializableHandler::Root* root,
    base::inplace_function<void ()> updated,
    const RegisterOptions& options) {
  PoolPtr<TypeErasedSerializableHandler> handler(
      &impl_->pool_, serializable, root);

  Impl::Element element;
  element.serializable = handler.get();
  element.updated = updated;
  element.enumerate = options.enumerate;

  const auto result = impl_->elements_.insert({name, element});
  // We do not allow duplicate names.
  MJ_ASSERT(result.second == true);
}

}
}
