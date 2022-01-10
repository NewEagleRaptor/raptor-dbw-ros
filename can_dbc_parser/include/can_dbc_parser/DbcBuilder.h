/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 New Eagle
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of New Eagle nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
#ifndef _NEW_EAGLE_DBCBUILDER_H
#define _NEW_EAGLE_DBCBUILDER_H

#include <ros/ros.h>

#include <string>

#include <can_dbc_parser/Dbc.h>

#include "LineParser.h"

namespace NewEagle
{
  struct DbcSignalValueType
  {
    uint32_t Id;
    std::string SignalName;
    NewEagle::DataType Type;
  };

  struct DbcAttribute
  {
    std::string AttributeName;
    uint32_t Id;
    std::string ObjectType;
    std::string SignalName;
    std::string Value;
  };

  class DbcBuilder
  {
    public:
      DbcBuilder();
      ~DbcBuilder();

      NewEagle::Dbc NewDbc(const std::string &dbcFile);

    private:
      std::string MessageToken;
      std::string SignalToken;
      std::string CommentToken;
      std::string EnumValueToken;
      std::string AttributeToken;
      std::string SignalValueTypeToken;
  };

  static NewEagle::DbcSignalValueType ReadSignalValueType(NewEagle::LineParser parser);
  static NewEagle::DbcAttribute ReadAttribute(NewEagle::LineParser parser);
  static NewEagle::DbcMessageComment ReadMessageComment(NewEagle::LineParser parser);
  static NewEagle::DbcSignalComment ReadSignalComment(NewEagle::LineParser parser);
  static NewEagle::DbcMessage ReadMessage(NewEagle::LineParser parser);
  static NewEagle::DbcSignal ReadSignal(NewEagle::LineParser parser);
}

#endif // _NEW_EAGLE_DBCBUILDER_H
