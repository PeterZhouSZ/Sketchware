/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/*                  This file is part of the class library                   */
/*       SoPlex --- the Sequential object-oriented simPlex.                  */
/*                                                                           */
/*    Copyright (C) 1996-2014 Konrad-Zuse-Zentrum                            */
/*                            fuer Informationstechnik Berlin                */
/*                                                                           */
/*  SoPlex is distributed under the terms of the ZIB Academic Licence.       */
/*                                                                           */
/*  You should have received a copy of the ZIB Academic License              */
/*  along with SoPlex; see the file COPYING. If not email to soplex@zib.de.  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "spxout.h"
#include "exceptions.h"
#include "spxalloc.h"

namespace soplex
{
   /// constructor
   SPxOut::SPxOut()
      : m_verbosity( ERROR )
      , m_streams(0)
   {
      spx_alloc(m_streams, INFO3+1);
      m_streams = new (m_streams) std::ostream*[INFO3+1];
      m_streams[ ERROR ] = m_streams[ WARNING ] = &std::cerr;
      for ( int i = DEBUG; i <= INFO3; ++i )
         m_streams[ i ] = &std::cout;
   }

   //---------------------------------------------------

   // destructor
   SPxOut::~SPxOut()
   {
      spx_free(m_streams);
   }

   //---------------------------------------------------

   // define global instance
   SPxOut spxout;

} // namespace soplex
