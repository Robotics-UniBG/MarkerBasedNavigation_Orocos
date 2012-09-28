/*
 * KUKA Youbot component for Orocos RTT
 *
 * (C) 2011 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
 *
 *            Department of Mechanical Engineering,
 *           Katholieke Universiteit Leuven, Belgium.
 *
 *  You may redistribute this software and/or modify it under either the
 *  terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1
 *  <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at you
 *  discretion) of the Modified BSD License:
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *  products derived from this software without specific prior written
 *  permission.
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIREC
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISIN
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <stdint.h>
#include <cstdio>
#include <rtt/Logger.hpp>

#include "youbot_types.hpp"

namespace YoubotDriver {
    void rtt_sleep(int ms);
    std::string control_mode2str(ControlMode cm);
    std::string& make_event(std::string& s, const char* event, int num);
}
