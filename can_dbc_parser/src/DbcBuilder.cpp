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

#include <can_dbc_parser/DbcBuilder.h>
#include <can_dbc_parser/DbcMessage.h>
#include <can_dbc_parser/DbcSignal.h>

namespace NewEagle
{
  DbcBuilder::DbcBuilder()
  {
    MessageToken = std::string("BO_");
    SignalToken = std::string("SG_");
    CommentToken = std::string("CM_");
    EnumValueToken = std::string("VAL_");
    AttributeToken = std::string("BA_");
    SignalValueTypeToken = std::string("SIG_VALTYPE_");
  }

  DbcBuilder::~DbcBuilder()
  {
  }

  NewEagle::Dbc DbcBuilder::NewDbc(const std::string& dbcFile)
  {
    NewEagle::Dbc dbc;

    std::istringstream f(dbcFile);
    std::string line;
    uint32_t lineNumber = 0;

    NewEagle::DbcMessage currentMessage;

    int32_t cnt = 0;

    while (std::getline(f, line, '\n'))
    {
      lineNumber++;

      NewEagle::LineParser parser(line);

      std::string identifier;
      try
      {
        identifier = parser.ReadCIdentifier();
      }
      catch (std::exception& ex)
      {
        identifier = std::string();
      }

      if (!MessageToken.compare(identifier))
      {
        try
        {
          currentMessage = ReadMessage(parser);
          currentMessage.SetRawText(line);
          dbc.AddMessage(currentMessage.GetName(), currentMessage);
        }
        catch (LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("DBC Message Parser Exception: [%s]", ex.what());
        }
      }
      else if (!SignalToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcSignal signal = ReadSignal(parser);

          NewEagle::DbcMessage* msg = dbc.GetMessage(currentMessage.GetName());
          msg->AddSignal(signal.GetName(), signal);
        }
        catch (LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("DBC Signal Parser Exception: [%s]", ex.what());
        }
      }
      else if (!CommentToken.compare(identifier))
      {
        try
        {
          std::string token = parser.ReadCIdentifier();

          if (!MessageToken.compare(token))
          {
            NewEagle::DbcMessageComment dbcMessageComment = ReadMessageComment(parser);

            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            int32_t ttt = 0;

            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              ttt++;
              if (it->second.GetRawId() == dbcMessageComment.Id)
              {
                it->second.SetComment(dbcMessageComment);
                break;
              }
            }
          }
          else if (!SignalToken.compare(token))
          {
            NewEagle::DbcSignalComment dbcSignalComment = ReadSignalComment(parser);

            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcSignalComment.Id)
              {
                DbcMessage msg = it->second;
                std::map<std::string, NewEagle::DbcSignal>::iterator its;

                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcSignalComment.SignalName)
                  {
                    its->second.SetComment(dbcSignalComment);
                    break;
                  }
                }
              }
            }
          }
        }
        catch (LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("DBC Comment Parser Exception: [%s]", ex.what());
        }
      }
      else if (!AttributeToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcAttribute dbcAttribute = ReadAttribute(parser);

          if (dbc.GetMessageCount() > 0)
          {
            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcAttribute.Id)
              {
                std::map<std::string, NewEagle::DbcSignal>::iterator its;
                DbcMessage msg = it->second;
                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcAttribute.SignalName)
                  {
                    NewEagle::DbcSignal sig = its->second;

                    double gain = sig.GetGain();
                    double offset = sig.GetOffset();

                    double f = 0.0;

                    std::stringstream ss;
                    ss << dbcAttribute.Value;
                    ss >> f;

                    double val = gain * f + offset;
                    sig.SetInitialValue(val);
                    break;
                  }
                }
              }
            }
          }
        }
        catch (LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("DBC Signal Value Type Parser Exception: [%s]", ex.what());
        }
      }
      else if (!EnumValueToken.compare(identifier))
      {
        // Empty for now.
      }
      else if (!SignalValueTypeToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcSignalValueType dbcSignalValueType = ReadSignalValueType(parser);

          if (dbc.GetMessageCount() > 0)
          {
            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcSignalValueType.Id)
              {
                std::map<std::string, NewEagle::DbcSignal>::iterator its;
                DbcMessage msg = it->second;
                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcSignalValueType.SignalName)
                  {
                    NewEagle::DbcSignal sig = its->second;
                    sig.SetDataType(dbcSignalValueType.Type);
                    break;
                  }
                }
              }
            }
          }
        }
        catch (LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("DBC Signal Value Type Parser Exception: [%s]", ex.what());
        }
      }
    }

    ROS_INFO("dbc.size() = %d", dbc.GetMessageCount());

    return dbc;
  }

  DbcSignalValueType ReadSignalValueType(LineParser parser)
  {
    NewEagle::DbcSignalValueType signalValueType;

    signalValueType.Id = parser.ReadUInt("id");
    signalValueType.SignalName = parser.ReadCIdentifier();
    parser.SeekSeparator(':');
    signalValueType.Type = parser.ReadUInt("DataType") == 1 ? NewEagle::FLOAT : NewEagle::DOUBLE;

    return signalValueType;
  }

  DbcAttribute ReadAttribute(LineParser parser)
  {
    NewEagle::DbcAttribute attribute;

    try
    {
      attribute.AttributeName = parser.ReadQuotedString();
      if (attribute.AttributeName == "GenSigStartValue")
      {
        attribute.ObjectType = parser.ReadCIdentifier();
        attribute.Id = parser.ReadUInt("id");
        if (attribute.ObjectType == "SG_")
        {
          attribute.SignalName = parser.ReadCIdentifier();

          std::ostringstream sstream;
          sstream << parser.ReadDouble();
          attribute.Value = sstream.str();
        }
      }
    }
    catch (std::exception& ex)
    {
      throw;
    }

    return attribute;
  }

  DbcMessageComment ReadMessageComment(LineParser parser)
  {
    NewEagle::DbcMessageComment comment;
    comment.Id = parser.ReadUInt("id");
    comment.Comment = parser.ReadQuotedString();

    return comment;
  }

  DbcSignalComment ReadSignalComment(LineParser parser)
  {
    NewEagle::DbcSignalComment comment;
    comment.Id = parser.ReadUInt("id");
    comment.SignalName = parser.ReadCIdentifier();
    comment.Comment = parser.ReadQuotedString();

    return comment;
  }

  DbcMessage ReadMessage(LineParser parser)
  {
    uint32_t canId = parser.ReadUInt("message id");
    IdType idType = ((canId & 0x80000000u) > 0) ? NewEagle::EXT : NewEagle::STD;
    std::string name(parser.ReadCIdentifier("message name"));
    parser.SeekSeparator(':');
    uint8_t dlc = parser.ReadUInt("size");
    std::string sendingNode(parser.ReadCIdentifier("transmitter"));
    uint32_t id = (uint32_t)(canId & 0x3FFFFFFFu);

    NewEagle::DbcMessage msg(dlc, id, idType, name, canId);
    return msg;
  }

  DbcSignal ReadSignal(LineParser parser)
  {
    std::string name = parser.ReadCIdentifier();
    char mux = parser.ReadNextChar("mux");
    NewEagle::MultiplexerMode multiplexMode = NewEagle::NONE;
    int32_t muxSwitch = 0;

    switch (mux)
    {
      case ':':
        multiplexMode = NewEagle::NONE;
        break;
      case 'M':
        multiplexMode = NewEagle::MUX_SWITCH;
        parser.SeekSeparator(':');
        break;
      case 'm':
        multiplexMode = NewEagle::MUX_SIGNAL;
        muxSwitch = parser.ReadInt();
        parser.SeekSeparator(':');
        break;
      default:
        throw std::runtime_error("Synxax Error: Expected \':\' " + parser.GetPosition());
    }

    int32_t startBit = parser.ReadUInt("start bit");

    parser.SeekSeparator('|');

    uint8_t length = (uint8_t)parser.ReadUInt("size");
    parser.SeekSeparator('@');

    char byteOrder = parser.ReadNextChar("byte order");

    NewEagle::ByteOrder endianness;

    switch (byteOrder)
    {
      case '1':
        endianness = NewEagle::LITTLE_END;
        break;
      case '0':
        endianness = NewEagle::BIG_END;
        break;
      default:
        throw std::runtime_error("Synxax Error: Byte Order - Expected 0 or 1, got " + byteOrder);
    }

    char valType = parser.ReadNextChar("value type");
    NewEagle::SignType sign;

    switch (valType)
    {
      case '+':
        sign = NewEagle::UNSIGNED;
        break;
      case '-':
        sign = NewEagle::SIGNED;
        break;
      default:
        throw std::runtime_error("Synxax Error: Value Type - Expected + or -, got " + valType);
    }

    // Set the default data type: INT.
    //  Might be changed what parsing SIG_VALTYPE_
    NewEagle::DataType type = NewEagle::INT;

    parser.SeekSeparator('(');
    double gain = parser.ReadDouble("factor");
    parser.SeekSeparator(',');
    double offset = parser.ReadDouble("offset");
    parser.SeekSeparator(')');

    parser.SeekSeparator('[');
    double minimum = parser.ReadDouble("minimum");
    parser.SeekSeparator('|');
    double maximum = parser.ReadDouble("maximum");
    parser.SeekSeparator(']');

    // Need to include Min, Max, DataType, MuxSwitch, Unit, Receiver
    // Find a way to include the DLC...
    NewEagle::DbcSignal* signal;

    if (NewEagle::MUX_SIGNAL == multiplexMode)
    {
      // NewEagle::DbcSignal signal(8, gain, offset, startBit, endianness, length, sign, name, multiplexMode, muxSwitch);
      signal =
          new NewEagle::DbcSignal(8, gain, offset, startBit, endianness, length, sign, name, multiplexMode, muxSwitch);
    }
    else
    {
      // NewEagle::DbcSignal signal(8, gain, offset, startBit, endianness, length, sign, name, multiplexMode);
      signal = new NewEagle::DbcSignal(8, gain, offset, startBit, endianness, length, sign, name, multiplexMode);
    }

    signal->SetDataType(type);
    return NewEagle::DbcSignal(*signal);
  }
}
