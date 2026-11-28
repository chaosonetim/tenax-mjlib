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
// Modified by Tenax Motion, 2026: one server can answer on several
// IDs, one register map per ID (AddServer), each with its own tunnels.

#pragma once

#include "mjlib/base/visitor.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"

#include "mjlib/multiplex/format.h"
#include "mjlib/multiplex/micro_datagram_server.h"

namespace mjlib {
namespace multiplex {

/// Implements a multi-node frame based packet protocol on top of an
/// AsyncStream.  This node's ID is stored in a PersistentConfig.
///
/// No dynamic memory is used.
class MicroServer : public Format {
 public:
  /// Applications implementing the server should provide a concrete
  /// implementation of this interface.  Within a single frame, all
  /// calls to Write or Read will take place before returning to the
  /// event loop.  Applications may use this fact to implement atomic
  /// updates as necessary.
  class Server {
   public:
    virtual ~Server() {}

    /// Called at the beginning of a frame.
    virtual void StartFrame() = 0;

    enum WriteAction {
      // Writing was a success.
      kSuccess,

      // The written register was unknown.
      kUnknownRegister,

      // The written register was known, but is not writeable.
      kNotWriteable,

      // Do not process any further data from this frame.
      kDiscardRemaining,
    };

    /// Attempt to store the given value.
    virtual WriteAction Write(Register, const Value&) = 0;

    /// @param type_index is an index into the Value variant
    /// describing what type to return.
    virtual ReadResult Read(Register, size_t type_index) const = 0;

    enum Action {
      // Accept the frame and respond accordingly.
      kAccept,

      // Discard the frame, do not respond in any way.
      kDiscard,
    };

    /// Called at the end of a frame.
    virtual Action CompleteFrame() = 0;
  };

  struct Options {
    int buffer_size = 256;
    int max_tunnel_streams = 1;
    uint8_t default_id = 1;

    /// The total number of register maps, including the one passed to
    /// Start.  Each one after the first is added with AddServer.
    int max_servers = 1;
  };

  MicroServer(micro::Pool*, MicroDatagramServer*, const Options&);
  ~MicroServer();

  /// Allocate a "tunnel", where an AsyncStream is tunneled over the
  /// multiplex connection.
  ///
  /// @param server if given, is a server that is (or will be) added
  /// with AddServer: the tunnel is then channel @p id of that
  /// server's ID rather than of the ID in config(), as if it were a
  /// separate device.  Each ID has its own channel numbers.
  micro::AsyncStream* MakeTunnel(uint32_t id,
                                 const Server* server = nullptr);

  void Start(Server*);

  /// Also answer frames sent to `*id` with `server`, as if it were a
  /// separate device.  `id` is read for every frame, so it may point
  /// into persistent configuration.  Its ID has the tunnels made
  /// with `server`; if there are none, tunnel subframes in its frames
  /// are ignored.  A broadcast frame is given to every server, and
  /// only the server passed to Start may reply to it or is given its
  /// tunnel subframes.  When two IDs match, the one in config() wins.
  ///
  /// At most Options::max_servers - 1 may be added.
  void AddServer(const uint8_t* id, Server* server);

  void Poll();

  // Exposed mostly for debugging and unit testing.
  struct Stats {
    uint32_t wrong_id = 0;
    uint32_t receive_overrun = 0;
    uint32_t unknown_subframe = 0;
    uint32_t missing_subframe = 0;
    uint32_t malformed_subframe = 0;
    uint32_t write_error = 0;
    uint32_t last_write_error = 0;
    uint32_t discards = 0;
    uint32_t response_buffer_full = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(wrong_id));
      a->Visit(MJ_NVP(receive_overrun));
      a->Visit(MJ_NVP(unknown_subframe));
      a->Visit(MJ_NVP(missing_subframe));
      a->Visit(MJ_NVP(malformed_subframe));
      a->Visit(MJ_NVP(write_error));
      a->Visit(MJ_NVP(last_write_error));
      a->Visit(MJ_NVP(discards));
      a->Visit(MJ_NVP(response_buffer_full));
    }
  };

  const Stats* stats() const;

  struct Config {
    uint8_t id = 1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
    }
  };

  Config* config();

 private:
  class Impl;
  micro::PoolPtr<Impl> impl_;
};

}
}
