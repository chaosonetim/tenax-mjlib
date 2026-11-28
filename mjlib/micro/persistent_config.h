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

#pragma once

#include "mjlib/base/inplace_function.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/flash.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/serializable_handler.h"

namespace mjlib {
namespace micro {

class PersistentConfig {
 public:
  /// @param max_elements is how many names may be registered.
  PersistentConfig(Pool&, CommandManager&, FlashInterface&,
                   mjlib::base::string_span,
                   size_t max_elements = 16);
  ~PersistentConfig();

  struct RegisterOptions {
    bool enumerate = true;

    RegisterOptions() {}
  };

  /// Associate the given serializable with the given name.
  ///
  /// Both @p name and @p serializable are aliased internally and must
  /// remain valid forever.
  template <typename Serializable>
  void Register(const std::string_view& name, Serializable* serializable,
                base::inplace_function<void ()> updated,
                const RegisterOptions& options = RegisterOptions()) {
    RegisterDetail(name, serializable, &kSerializableRoot<Serializable>,
                   updated, options);
  }

  /// Also serve the "conf" commands on @p command_manager, as a
  /// device would that had only the groups whose names start with
  /// @p prefix, each without that prefix.  So with a view for
  /// "axisb_", its "conf set servo.x 1" sets "axisb_servo.x".
  ///
  /// A view lists its own groups, and each group that starts with no
  /// view's prefix unless one of its own has that name.  Once any view
  /// is added, the constructor's command manager no longer lists the
  /// groups of the views.  Every command manager still reaches any
  /// group by its full name.
  ///
  /// "conf default" restores the groups of its view, and from the
  /// constructor's command manager those of no view as well.  "conf
  /// load", "conf write" and "conf size" are for every group whichever
  /// command manager they come from, since there is one store.
  ///
  /// @p prefix is aliased and must remain valid forever, and may not
  /// be empty.  @p output_buffer may not be the buffer of any other
  /// command manager whose commands can be in progress at the same
  /// time.
  void AddView(CommandManager& command_manager,
               const std::string_view& prefix,
               mjlib::base::string_span output_buffer);

  /// Restore all registered configuration structures from Flash.
  /// This should be invoked after all modules have had a chance to
  /// register their configurables.
  void Load();

 private:
  /// This aliases @p serializable, which must remain valid for the
  /// lifetime of the PersistentConfig.
  void RegisterDetail(const std::string_view& name,
                      void* serializable,
                      const TypeErasedSerializableHandler::Root*,
                      base::inplace_function<void ()> updated,
                      const RegisterOptions&);

  class Impl;
  PoolPtr<Impl> impl_;
};

}
}
