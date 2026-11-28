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
// Modified by Tenax Motion, 2026: tests of more registered names than
// the default, and of views.

#include "mjlib/micro/persistent_config.h"

#include <boost/test/auto_unit_test.hpp>

#include "mjlib/micro/test/persistent_config_fixture.h"
#include "mjlib/micro/test/str.h"

using namespace mjlib::micro;

using test::str;

namespace {
struct Fixture : test::PersistentConfigFixture {
  PersistentConfig& dut = persistent_config;

  int my_data_count = 0;
  int other_data_count = 0;

  test::MyData non_enumerated;
  int non_enumerated_count = 0;

  Fixture() {
    dut.Register("my_data", &my_data, [this]() {
        this->my_data_count++;
      });
    dut.Register("other_data", &other_data, [this]() {
        this->other_data_count++;
      });
    PersistentConfig::RegisterOptions register_options;
    register_options.enumerate = false;
    dut.Register("non_enumerated", &non_enumerated, [this]() {
        this->non_enumerated_count++;
      },
      register_options);
  }
};
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigEnumerate, Fixture) {
  Command("conf enumerate\n");
  ExpectResponse("my_data.value 0\r\nother_data.stuff 0\r\nOK\r\n");
}

namespace {
struct EmptyStruct {
  template <typename Archive>
  void Serialize(Archive*) {}
};

// Tight-buffer fixture: prefix+name+CRLF exactly fills the output
// buffer. Pre-fix this caused Emit to skip the 4-byte size
// placeholder past the buffer end, then write the (zero-byte)
// length there, corrupting whatever lived after the buffer.
// Post-fix it returns "ERR name too long" via WriteMessage, and
// the re-enabled skip() assert in BufferWriteStream catches any
// remaining future violation immediately.
struct TightBufferFixture : test::CommandManagerFixture {
  test::StubFlash flash;
  // 16 bytes is exactly the size of "cdata " + "abcdefgh" + "\r\n".
  char output_buffer[16] = {};
  PersistentConfig persistent_config{
    pool, command_manager, flash,
    mjlib::base::string_span(output_buffer)};
  EmptyStruct empty;

  TightBufferFixture() {
    persistent_config.Register("abcdefgh", &empty, []() {});
  }
};
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigEmitNameTooLong, TightBufferFixture) {
  Command("conf data abcdefgh\n");
  ExpectResponse("ERR name too long\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigEnumerateGroup1, Fixture) {
  Command("conf enumerate my_data\n");
  ExpectResponse("my_data.value 0\r\nOK\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigEnumerateGroup2, Fixture) {
  Command("conf enumerate other_data\n");
  ExpectResponse("other_data.stuff 0\r\nOK\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigEnumerateGroupUnknown, Fixture) {
  Command("conf enumerate notfound\n");
  ExpectResponse("ERR unknown group\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigList, Fixture) {
  Command("conf list\n");
  ExpectResponse("my_data\r\nother_data\r\nnon_enumerated\r\nOK\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigSchema, Fixture) {
  Command("conf schema my_data\n");
  ExpectResponsePrefix("cschema my_data\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigData, Fixture) {
  Command("conf data my_data\n");
  ExpectResponsePrefix("cdata my_data\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigGet, Fixture) {
  Command("conf get my_data.value\n");
  ExpectResponse("0\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigSet, Fixture) {
  BOOST_TEST(my_data.value == 0);
  Command("conf set my_data.value 39\n");
  ExpectResponse("OK\r\n");
  BOOST_TEST(my_data.value == 39);

  Command("conf set non_enumerated.value 47\n");
  ExpectResponse("OK\r\n");
  BOOST_TEST(non_enumerated.value == 47);
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigFlash, Fixture) {
  my_data.value = 76;
  other_data.stuff = 23;
  Command("conf write\n");
  ExpectResponse("OK\r\n");

  BOOST_TEST(my_data.value == 76);
  BOOST_TEST(other_data.stuff == 23);
  BOOST_TEST(my_data_count == 0);

  Command("conf default\n");
  ExpectResponse("OK\r\n");

  BOOST_TEST(my_data.value == 0);
  BOOST_TEST(other_data.stuff == 0);

  BOOST_TEST(my_data_count == 1);
  Command("conf load\n");
  ExpectResponse("OK\r\n");

  BOOST_TEST(my_data_count == 2);
  BOOST_TEST(my_data.value == 76);
  BOOST_TEST(other_data.stuff == 23);

  Command("conf size\n");
  ExpectResponse("72\r\nOK\r\n");
}

namespace {
constexpr int kManyNames = 20;
constexpr const char* kNames[kManyNames] = {
  "n00", "n01", "n02", "n03", "n04", "n05", "n06", "n07", "n08", "n09",
  "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19",
};

struct ManyFixture : test::CommandManagerFixture {
  test::StubFlash flash;
  char output_buffer[4096] = {};
  // More than the default of 16.
  PersistentConfig dut{
    pool, command_manager, flash,
    mjlib::base::string_span(output_buffer), kManyNames};
  test::MyData data[kManyNames];

  ManyFixture() {
    for (int i = 0; i < kManyNames; i++) {
      dut.Register(kNames[i], &data[i], []() {});
    }
  }
};
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigMaxElements, ManyFixture) {
  Command("conf get n19.value\n");
  ExpectResponse("0\r\n");
}

namespace {
// A second command manager with a view, as a second axis of one board
// would have, alongside the first.
struct ViewFixture : test::CommandManagerFixture {
  test::StubFlash flash;
  char output_buffer[4096] = {};
  PersistentConfig dut{
    pool, command_manager, flash,
    mjlib::base::string_span(output_buffer)};

  SizedPool<> pool_b;
  StreamPipe pipe_b{event_queue.MakePoster()};
  test::Reader reader_b{pipe_b.side_b()};
  AsyncExclusive<AsyncWriteStream> write_stream_b{pipe_b.side_a()};
  CommandManager command_manager_b{
    &pool_b, pipe_b.side_a(), &write_stream_b};
  char output_buffer_b[4096] = {};

  // "servo" and "id" are per axis, "board" is shared by both, and
  // "can" only the first axis has.
  test::MyData servo;
  test::MyData servo_b;
  test::OtherData id;
  test::OtherData id_b;
  test::MyData board;
  test::OtherData hidden_b;

  int servo_count = 0;
  int servo_b_count = 0;
  int board_count = 0;

  ViewFixture() {
    command_manager_b.AsyncStart();

    dut.Register("servo", &servo, [this]() { servo_count++; });
    dut.Register("id", &id, []() {});
    dut.Register("board", &board, [this]() { board_count++; });
    dut.Register("axisb_servo", &servo_b, [this]() { servo_b_count++; });
    dut.Register("axisb_id", &id_b, []() {});
    PersistentConfig::RegisterOptions register_options;
    register_options.enumerate = false;
    dut.Register("axisb_hidden", &hidden_b, []() {}, register_options);

    dut.AddView(command_manager_b, "axisb_",
                mjlib::base::string_span(output_buffer_b));
  }

  void CommandB(const std::string_view& message) {
    RequiredSuccess required_success;
    AsyncWrite(*pipe_b.side_b(), message, required_success.Make());
    event_queue.Poll();
  }

  void ExpectResponseB(const std::string_view& message) {
    BOOST_TEST(reader_b.data_.str() == message);
    reader_b.data_.str("");
    // Nothing is ever said to the other command manager.
    BOOST_TEST(reader.data_.str() == "");
  }
};
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewEnumerate, ViewFixture) {
  servo.value = 1;
  servo_b.value = 2;
  id.stuff = 3;
  id_b.stuff = 4;
  board.value = 5;

  // The first command manager no longer lists what belongs to a view.
  Command("conf enumerate\n");
  ExpectResponse("servo.value 1\r\nid.stuff 3\r\nboard.value 5\r\nOK\r\n");

  // The view lists its own groups by their plain names, in place of
  // the first's, and what both share.
  CommandB("conf enumerate\n");
  ExpectResponseB("board.value 5\r\nservo.value 2\r\nid.stuff 4\r\nOK\r\n");

  CommandB("conf enumerate servo\n");
  ExpectResponseB("servo.value 2\r\nOK\r\n");

  CommandB("conf enumerate board\n");
  ExpectResponseB("board.value 5\r\nOK\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewList, ViewFixture) {
  Command("conf list\n");
  ExpectResponse("servo\r\nid\r\nboard\r\nOK\r\n");

  CommandB("conf list\n");
  ExpectResponseB("board\r\nservo\r\nid\r\nhidden\r\nOK\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewGetSet, ViewFixture) {
  CommandB("conf set servo.value 12\n");
  ExpectResponseB("OK\r\n");
  BOOST_TEST(servo.value == 0);
  BOOST_TEST(servo_b.value == 12);
  BOOST_TEST(servo_count == 0);
  BOOST_TEST(servo_b_count == 1);

  CommandB("conf get servo.value\n");
  ExpectResponseB("12\r\n");

  Command("conf get servo.value\n");
  ExpectResponse("0\r\n");

  // What both share is the same from either.
  CommandB("conf set board.value 7\n");
  ExpectResponseB("OK\r\n");
  BOOST_TEST(board.value == 7);
  BOOST_TEST(board_count == 1);
  Command("conf get board.value\n");
  ExpectResponse("7\r\n");

  // Groups that are not enumerated still work by name.
  CommandB("conf set hidden.stuff 9\n");
  ExpectResponseB("OK\r\n");
  BOOST_TEST(hidden_b.stuff == 9);

  CommandB("conf get nothing.value\n");
  ExpectResponseB("ERR unknown group\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewFullNames, ViewFixture) {
  // The full name of any group works from every command manager.
  Command("conf set axisb_servo.value 21\n");
  ExpectResponse("OK\r\n");
  BOOST_TEST(servo_b.value == 21);

  Command("conf enumerate axisb_servo\n");
  ExpectResponse("axisb_servo.value 21\r\nOK\r\n");

  CommandB("conf get axisb_servo.value\n");
  ExpectResponseB("21\r\n");
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewSchemaData, ViewFixture) {
  servo.value = 0x11111111;
  servo_b.value = 0x22222222;

  CommandB("conf schema servo\n");
  BOOST_TEST(reader_b.data_.str().substr(0, 15) == "cschema servo\r\n");
  reader_b.data_.str("");

  CommandB("conf data servo\n");
  const std::string expected =
      std::string("cdata servo\r\n") +
      std::string("\x04\x00\x00\x00\x22\x22\x22\x22", 8);
  ExpectResponseB(expected);
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewDefault, ViewFixture) {
  servo.value = 1;
  servo_b.value = 2;
  board.value = 3;

  // A view restores only its own.
  CommandB("conf default\n");
  ExpectResponseB("OK\r\n");
  BOOST_TEST(servo.value == 1);
  BOOST_TEST(servo_b.value == 0);
  BOOST_TEST(board.value == 3);
  BOOST_TEST(servo_count == 0);
  BOOST_TEST(servo_b_count == 1);

  // The first restores its own and what is shared.
  servo_b.value = 2;
  Command("conf default\n");
  ExpectResponse("OK\r\n");
  BOOST_TEST(servo.value == 0);
  BOOST_TEST(servo_b.value == 2);
  BOOST_TEST(board.value == 0);
  BOOST_TEST(servo_count == 1);
  BOOST_TEST(servo_b_count == 1);
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewFlash, ViewFixture) {
  servo.value = 31;
  servo_b.value = 32;

  // There is one store, written and read whole from either.
  CommandB("conf write\n");
  ExpectResponseB("OK\r\n");

  servo.value = 0;
  servo_b.value = 0;

  CommandB("conf load\n");
  ExpectResponseB("OK\r\n");
  BOOST_TEST(servo.value == 31);
  BOOST_TEST(servo_b.value == 32);
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigViewInterleaved, ViewFixture) {
  servo.value = 1;
  servo_b.value = 2;
  id.stuff = 3;
  id_b.stuff = 4;
  board.value = 5;

  // Hold back what the first command manager writes, so that its
  // enumeration is still in progress while the view runs one whole.
  reader.AllowReads(0);
  event_queue.Poll();
  Command("conf enumerate\n");

  CommandB("conf enumerate\n");
  BOOST_TEST(reader_b.data_.str() ==
             "board.value 5\r\nservo.value 2\r\nid.stuff 4\r\nOK\r\n");

  reader.AllowReads(-1);
  event_queue.Poll();
  ExpectResponse("servo.value 1\r\nid.stuff 3\r\nboard.value 5\r\nOK\r\n");
}
