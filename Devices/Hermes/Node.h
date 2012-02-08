
namespace rpg
{

	class Node
	{

		Node()
		{
			m_pContext = new zmq::context_t(1);;
		}


		~Node()
		{
			std::map < std::string, zmq::socket_t* >::iterator it;

			for( it = m_mEndpoints.begin(); it != m_mEndpoints.end(); it++ ) {
				delete (*it).second;
				m_mEndpoints.erase( it );
			}
		}

		void Publish(  )
		{

		}


		void Subscribe(  )
		{

		}

		void Replies(  )
		{

		}

		void Requests(  )
		{

		}

/*		
	node n;

	n.Subscribe( host, port, pb );

	n.CallService( host1, port, "Func1", req, rep );
	n.CallService( host1, port, "Func2", req, rep );
	n.CallService( host2, port, "Func", req, rep );

	while(1){
             n.Read( pb );
        }

        ///////////////// 

	void f( req, rep )
	{

	}

	node n( host );

	n.Publishes( port, pb );
	n.ProvidesService( port, "Func1", f1, req, rep );
	n.ProvidesService( port, "Func2", f2, req, rep );





		bool AddEndpoint( std::string Node, EndpointType eType  ) {
			std::pair< std::map < std::string, zmq::socket_t* >::iterator, bool > ret;

			zmq::socket_t* Socket = new zmq::socket_t( *m_pContext, eType );

			ret = m_mEndpoints.insert ( std::pair < std::string, zmq::socket_t* > ( Node, Socket ) );
			return ret.second;
		}


		bool RemoveEndpoint( std::string Node ) {
			std::map < std::string, zmq::socket_t* >::iterator ep;

			ep = m_mEndpoints.find( Node );
			if( ep != m_mEndpoints.end() ) {
				delete (*ep).second;
				m_mEndpoints.erase( ep );
				return true;
			} else {
				return false;
			}
		}


		bool Bind( std::string Node, std::string Host ) {
			std::map < std::string, zmq::socket_t* >::iterator ep;

			ep = m_mEndpoints.find( Node );
			if( ep == m_mEndpoints.end() ) {
				return false;
			} else {
				zmq::socket_t* pSocket = (*ep).second;
				try {
					pSocket->bind( Host.c_str() );
				}
				catch( zmq::error_t error ) {
					return false;
				}
				return true;
			}

		}


		bool Connect( std::string Node, std::string Host ) {
			std::map < std::string, zmq::socket_t* >::iterator ep;

			ep = m_mEndpoints.find( Node );
			if( ep == m_mEndpoints.end() ) {
				return false;
			} else {
				zmq::socket_t* pSocket = (*ep).second;
				try {
					pSocket->connect( Host.c_str() );
				}
				catch( zmq::error_t error ) {
					return false;
				}
				return true;
			}

		}


		bool Send( std::string Node, char *pData, unsigned int nData ) {
			std::map < std::string, zmq::socket_t* >::iterator ep;

			ep = m_mEndpoints.find( Node );
			if( ep == m_mEndpoints.end() ) {
				return false;
			} else {
				zmq::socket_t* pSocket = (*ep).second;
				zmq::message_t message( nData );
				std::memcpy( message.data(), pData, nData );
				pSocket->send( message );
				return true;
			}

		}
*/

	};
}
