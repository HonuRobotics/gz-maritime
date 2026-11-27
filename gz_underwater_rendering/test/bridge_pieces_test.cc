/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// The bridge's reading of Ogre's pieces, without Ogre running: how it
// chains our pieces into gz-rendering's, and every case it refuses. The
// bridge's source is compiled in, to reach what it keeps to itself.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "../src/systems/UnderwaterOgre2Bridge.cc"

namespace
{
/// \brief An archive of one file.
std::unique_ptr<MemoryArchive> Archive(const std::string &_name,
                                       const std::string &_text)
{
  auto a = std::make_unique<MemoryArchive>(_name);
  a->files[_name + ".any"] = _text;
  return a;
}

/// \brief The text of every file of a library.
std::string Text(const Ogre::ArchiveVec &_library)
{
  std::string text;
  for (auto *archive : _library)
  {
    const Ogre::StringVectorPtr names = archive->list(false, false);
    for (const auto &name : *names)
      text += archive->open(name)->getAsString();
  }
  return text;
}

const std::vector<std::string> kHooks{"custom_a", "custom_b"};
}  // namespace

/////////////////////////////////////////////////
TEST(Pieces, DefinitionsWithAnySpacing)
{
  EXPECT_EQ(1u, Definitions("@piece( custom_a )", "custom_a").size());
  EXPECT_EQ(1u, Definitions("@piece(custom_a)", "custom_a").size());
  EXPECT_EQ(1u, Definitions("@piece(  custom_a\t)", "custom_a").size());
  EXPECT_TRUE(Definitions("@piece( custom_ab )", "custom_a").empty());
  EXPECT_TRUE(Definitions("@insertpiece( custom_a )", "custom_a").empty())
      << "an insertpiece is not a definition";
  EXPECT_EQ(2u, Definitions("@piece( custom_a )\n@end\n@piece(custom_a)",
                            "custom_a").size());
}

/////////////////////////////////////////////////
// A piece that only inserts others takes our line before its @end; one
// that does anything else is refused, as is one with no @end.
TEST(Pieces, InsertOnlyIntoComposites)
{
  const std::string line = "@insertpiece( custom_a_underwater )";
  for (const std::string body :
       {"\n@insertpiece( x )\n", "\n  @insertpiece(x)\n\n  // why\n", "\n"})
  {
    std::string text = "@piece( custom_a )" + body + "@end\n";
    const auto at = Definitions(text, "custom_a").front();
    ASSERT_TRUE(InsertIntoComposite(text, at.first, at.second, line)) << body;
    EXPECT_NE(text.find(line + "\n@end"), std::string::npos) << text;
  }
  for (const std::string body :
       {"\n  outColour = vec4(1);\n", "\n@insertpiece( x ) float y;\n",
        "\n@property( foo )\n@insertpiece( x )\n@end\n"})
  {
    std::string text = "@piece( custom_a )" + body + "@end\n";
    const auto at = Definitions(text, "custom_a").front();
    EXPECT_FALSE(InsertIntoComposite(text, at.first, at.second, line))
        << body;
  }
  std::string open = "@piece( custom_a )\n@insertpiece( x )\n";
  EXPECT_FALSE(InsertIntoComposite(open, 0, 18, line));
}

/////////////////////////////////////////////////
// Chaining: a hook gz-rendering defines gets our line in a copy of its
// archive, the original untouched; a hook nobody defines is defined by us,
// last.
TEST(Pieces, ChainCopiesAndDefines)
{
  auto gz = Archive("gz", "@piece( custom_a )\n@insertpiece( gz_a )\n@end\n");
  auto other = Archive("other", "@piece( unrelated )\nfoo\n@end\n");
  Ogre::ArchiveVec library{gz.get(), other.get()};
  std::vector<std::unique_ptr<MemoryArchive>> owned;
  ASSERT_TRUE(Chain(library, kHooks, "underwater", owned));

  ASSERT_EQ(3u, library.size());
  EXPECT_NE(gz.get(), library[0]) << "the original was changed, not copied";
  EXPECT_EQ(other.get(), library[1]) << "an archive with no hook was copied";
  EXPECT_EQ(std::string::npos,
            gz->files["gz.any"].find("custom_a_underwater"));
  const std::string text = Text(library);
  EXPECT_NE(std::string::npos, text.find(
      "@insertpiece( gz_a )\n  @insertpiece( custom_a_underwater )\n@end"));
  EXPECT_NE(std::string::npos, text.find(
      "@piece( custom_b )\n  @insertpiece( custom_b_underwater )\n@end"));
  EXPECT_EQ(2u, owned.size());
}

/////////////////////////////////////////////////
// A hook defined twice, in one archive or across two, or defined by code
// we cannot extend: refused, the library unchanged.
TEST(Pieces, ChainRefuses)
{
  const std::string twice =
      "@piece( custom_a )\n@end\n@piece( custom_a )\n@end\n";
  auto one = Archive("one", twice);
  auto a = Archive("a", "@piece( custom_a )\n@end\n");
  auto b = Archive("b", "@piece(custom_a)\n@end\n");
  auto code = Archive("code", "@piece( custom_b )\n  x = 1.0;\n@end\n");
  for (Ogre::ArchiveVec library :
       {Ogre::ArchiveVec{one.get()}, Ogre::ArchiveVec{a.get(), b.get()},
        Ogre::ArchiveVec{code.get()}})
  {
    const Ogre::ArchiveVec before = library;
    std::vector<std::unique_ptr<MemoryArchive>> owned;
    EXPECT_FALSE(Chain(library, kHooks, "underwater", owned));
    EXPECT_EQ(before, library);
  }
}

/////////////////////////////////////////////////
// Our pieces must define each hook's piece, or nothing is installed.
TEST(Pieces, OursMustDefineEveryHook)
{
  auto all = Archive("ours",
      "@piece( custom_a_underwater )\n@end\n"
      "@piece( custom_b_underwater )\n@end\n");
  auto some = Archive("ours", "@piece( custom_a_underwater )\n@end\n");
  EXPECT_TRUE(Defines(all.get(), kHooks, "underwater"));
  EXPECT_FALSE(Defines(some.get(), kHooks, "underwater"));
}

/////////////////////////////////////////////////
// The pieces shipped with the package define every hook the bridge uses.
TEST(Pieces, ShippedPiecesDefineEveryHook)
{
  // Read as plain files: no Ogre archive manager without Ogre
  auto ours = std::make_unique<MemoryArchive>("shipped");
  for (const auto &entry : std::filesystem::directory_iterator(PIECES_SRC))
  {
    std::ifstream in(entry.path());
    ours->files[entry.path().filename().string()] = std::string(
        std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  EXPECT_TRUE(Defines(ours.get(), kPbsHooks, "underwater"));
}

/////////////////////////////////////////////////
// The caustics' phases, wrapped in double before the shader's float: the
// same pattern as the unwrapped phase however long the world has run, and
// no jump from one frame to the next.
TEST(Caustics, PhaseWrapsWithoutAJump)
{
  for (const double rate : {0.8, 1.0, 0.9, 1.1})
  {
    for (const double t : {0.0, 999.95, 1000.0, 1000.05, 12345.678, 1e6 + 0.7})
    {
      const float p = CausticPhase(rate, t);
      EXPECT_GE(p, 0.0f);
      EXPECT_LT(p, 6.2832f);
      EXPECT_NEAR(std::sin(p), std::sin(rate * t), 1e-5) << rate << " " << t;
      EXPECT_NEAR(std::cos(p), std::cos(rate * t), 1e-5) << rate << " " << t;
    }
  }
}
