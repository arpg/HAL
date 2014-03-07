#include <OpenStreetMap.h>

namespace osm
{

  //////////////////////////////////////////////////////////////////////////////
  double LatitudeToTileY(double lat, int zoom)
  {
    int shift = 1 << zoom;
    double yoffset = (8.0/(1 << (18 - zoom)))/256.0;
    double aux = log( tan( lat * M_PI / 180.0 ) + 1.0/cos( lat * M_PI / 180.0 ) );
    return (( 1.0 - aux / M_PI ) / 2.0) * shift + yoffset;
  }

  //////////////////////////////////////////////////////////////////////////////
  double LongitudeToTileX(double lon, int zoom)
  {
    int    shift = 1 << zoom;

    return (( lon + 180.0 ) / 360.0) * shift;
  }

  //////////////////////////////////////////////////////////////////////////////
  double TileYToLatitude(int y, int zoom)
  {
    double n = M_PI - 2.0 * M_PI * y / (1 << zoom);
    return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
  }

  //////////////////////////////////////////////////////////////////////////////
  double TileXToLongitude(int x, int zoom)
  {
    return (double)x / (1 << zoom) * 360.0 - 180;
  }


  //////////////////////////////////////////////////////////////////////////////
  // Compute gps point tile coordinate and relative position
  void CreatePoint(GPSPoint &pnt, int zoom, GPSPoint origin, float resolution)
  {
    int    shift = 1 << zoom;
    double yoffset = (8.0/(1 << (18 - zoom)))/256.0;
    double lat  = pnt.lat;
    double lon  = pnt.lon;
    double aux  = log( tan( lat * M_PI / 180.0 ) + 1.0/cos( lat * M_PI / 180.0 ) );
    double xpos = (( lon + 180.0 ) / 360.0) * shift;
    double ypos = (( 1.0 - aux / M_PI ) / 2.0) * shift + yoffset;

    double x,y;
    double ox = origin.tx + origin.xpos;
    double oy = origin.ty + origin.ypos;
    double tileSize   = resolution * 255;

    pnt.tx   = (int)xpos;
    pnt.ty   = (int)ypos;

    // compute location of point in the tile
    pnt.xpos = xpos - pnt.tx;
    pnt.ypos = ypos - pnt.ty;

    x = (LongitudeToTileX(pnt.lon,zoom) - ox)*tileSize;
    y = (oy - LatitudeToTileY(pnt.lat,zoom) )*tileSize;

    pnt.x =  x;
    pnt.y =  y;

    pnt.active = false;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Compute gps point tile coordinate and relative position
  void CreatePointNoOrg(GPSPoint &pnt, int zoom)
  {
    int    shift = 1 << zoom;
    double yoffset = (8.0/(1 << (18 - zoom)))/256.0;
    double lat   = pnt.lat;
    double lon   = pnt.lon;
    double aux   = log( tan( lat * M_PI / 180.0 ) + 1.0/cos( lat * M_PI / 180.0 ) );
    double xpos  = (( lon + 180.0 ) / 360.0) * shift;
    double ypos  = (( 1.0 - aux / M_PI ) / 2.0) * shift +yoffset;

    pnt.tx   = (int)xpos;
    pnt.ty   = (int)ypos;

    // compute location of point in the tile
    pnt.xpos = xpos - pnt.tx;
    pnt.ypos = ypos - pnt.ty;

    pnt.x =  0;
    pnt.y =  0;

    pnt.active = false;
  }

  //////////////////////////////////////////////////////////////////////////////
  size_t my_dummy_write(char *ptr, size_t size, size_t nmemb, void *userdata)
  {
    std::vector<char>& bytes = *((std::vector<char>*)userdata);
    bytes.insert( bytes.end(), (char*)ptr, (char*)ptr+size*nmemb );
    return size * nmemb;
  }

  //////////////////////////////////////////////////////////////////////////////
  template<typename T>
  T GetAttrib(rpg::TiXmlNode* xml_node, const std::string attr )
  {
    T t;
    xml_node->ToElement()->QueryValueAttribute( attr.c_str(), &t );
    return t;
  }


  //////////////////////////////////////////////////////////////////////////////
  void ParseNode(rpg::TiXmlNode* xml_node, osm::node& n)
  {
    n.set_id( GetAttrib<int>( xml_node, "id" ) );
    n.set_lat( GetAttrib<float>( xml_node, "lat" ) );
    n.set_lon( GetAttrib<float>( xml_node, "lon" ) );
    if( xml_node->ToElement()->Attribute("alt" )){
      n.set_alt( GetAttrib<float>( xml_node, "alt" ) );
    }
    if( xml_node->ToElement()->Attribute("user" )){
      n.set_user( GetAttrib<std::string>( xml_node, "user" ) );
    }
    if( xml_node->ToElement()->Attribute("uid" )){
      n.set_uid( GetAttrib<int>( xml_node, "uid" ) );
    }
    if( xml_node->ToElement()->Attribute("visible" )){
      n.set_visible( GetAttrib<bool>( xml_node, "visible" ) );
    }
    if( xml_node->ToElement()->Attribute("version" )){
      n.set_version( GetAttrib<int>( xml_node, "version" ) );
    }
    if( xml_node->ToElement()->Attribute("changeset" )){
      n.set_changeset( GetAttrib<int>( xml_node, "changeset" ) );
    }
    if( xml_node->ToElement()->Attribute("timestamp" )){
      n.set_timestamp( GetAttrib<std::string>( xml_node, "timestamp" ) );
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void ParseWay(rpg::TiXmlNode* way_node, osm::way& way_pb)
  {
    // loop through the XML nodes
    rpg::TiXmlNode* pNode = way_node->FirstChild( "nd" );
    for(; pNode; pNode = pNode->NextSibling("nd") ){
      int nNodeId = GetAttrib<int>( pNode, "ref" ); // use our XML helper
      way_pb.add_nodes( nNodeId );  // add to the repeated nodes field in the PB

    }
    way_pb.set_id( GetAttrib<int>(way_node, "id" ) );


    rpg::TiXmlNode* pTag = way_node->FirstChild( "tag" );
    for(; pTag; pTag = pTag->NextSibling("tag") ){
      std::string sKey = GetAttrib<std::string>( pTag, "k" ); // use our XML helper
      std::string sVal = GetAttrib<std::string>( pTag, "v" );
      osm::tag* pTagPB = way_pb.add_tags();
      pTagPB->set_key(sKey);
      pTagPB->set_val(sVal);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void ParseRelation(rpg::TiXmlNode* relation_node, osm::relation& relation_pb)
  {
    relation_pb.set_id( GetAttrib<int>( relation_node, "id" ) );
  }


  //////////////////////////////////////////////////////////////////////////////
  /// Primary user function for getting open street map data.
  void FetchOpenStreetMapData(const std::string           url,
                              std::vector<osm::node>&     nodes,
                              std::vector<osm::way>&      ways,
                              std::vector<osm::relation>& relations
                              )
  {
    std::vector<char> vBuf;
    CURL *curl;
    CURLcode res;

    curl = curl_easy_init();

    curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, &my_dummy_write );
    curl_easy_setopt( curl, CURLOPT_WRITEDATA, &vBuf );

    if( curl ){
      curl_easy_setopt( curl, CURLOPT_URL, url.c_str() );

      res = curl_easy_perform( curl );  // Perform the request, res will get the return code
      if( res != CURLE_OK ) {
        fprintf( stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
      }

      curl_easy_cleanup( curl );

      rpg::TiXmlDocument doc;
      doc.Parse( &vBuf[0] );

      rpg::TiXmlElement* pOSMNode = doc.FirstChildElement( "osm" );
      std::string sVer( pOSMNode->Attribute("version") );

      //cout << "Got version " << sVer << endl;

      rpg::TiXmlNode* pBoundsNode = pOSMNode->FirstChild( "bounds" );
      osm::bounds bounds;
      bounds.set_minlat( GetAttrib<double>( pBoundsNode, "minlat" ) );
      bounds.set_maxlat( GetAttrib<double>( pBoundsNode, "maxlat" ) );
      bounds.set_minlon( GetAttrib<double>( pBoundsNode, "minlon" ) );
      bounds.set_maxlon( GetAttrib<double>( pBoundsNode, "maxlon" ) );

      //printf("Bounds lat[%f:%f], lon[%f:%f]\n",
      //        bounds.minlat(), bounds.maxlat(), bounds.minlon(), bounds.maxlon() );

      // loop over nodes
      rpg::TiXmlNode* pNode = pOSMNode->FirstChild( "node" );
      for(; pNode; pNode = pNode->NextSibling("node") ){
        osm::node n;
        ParseNode( pNode, n );
        nodes.push_back( n );
      }

      // loop over ways
      rpg::TiXmlNode* pWay = pOSMNode->FirstChild( "way" );
      for(; pWay; pWay = pWay->NextSibling("way") ){
        osm::way w;
        ParseWay( pWay, w );
        ways.push_back( w );
      }
      // loop over relations
      rpg::TiXmlNode* pRelation = pOSMNode->FirstChild( "relation" );
      for(; pRelation; pRelation = pRelation->NextSibling("relation") ){
        osm::relation r;
        ParseRelation( pRelation, r );
        relations.push_back( r );
      }

    }

  }
  //////////////////////////////////////////////////////////////////////////////
  void ParseOpenStreetMapData(GPSPoint origin,
                              float resolution,
                              int zoom,
                              const std::vector<node>& nodes,
                              const std::vector<way>&  ways,
                              const std::vector<relation>& relations,
                              std::vector<Way>& ways_out
                              )
  {
    GPSPoint pnt;
    Eigen::Vector3i tile;
    int flag = 0;

    ways_out.resize(ways.size());

    for (unsigned int ii = 0; ii < ways.size() ; ii++) {
      //set ID of the way
      ways_out[ii].wayID = ways[ii].id();
      //set the vector of GPS points
      for (int jj=0; jj < ways[ii].nodes_size(); jj++) {

        for (unsigned int kk=0; kk< nodes.size(); kk++) {
          if(ways[ii].nodes(jj) == nodes[kk].id()) {
            pnt.lat = nodes[kk].lat();
            pnt.lon = nodes[kk].lon();
            CreatePoint(pnt,zoom,origin,resolution);
            ways_out[ii].waypoints.push_back(pnt);
            tile << ways_out[ii].waypoints[jj].tx,ways_out[ii].waypoints[jj].ty,0;

            for (size_t mm = 0; mm < ways_out[ii].tiles.size(); mm++) {
              if((ways_out[ii].tiles[mm](0) == tile(0)) && (ways_out[ii].tiles[mm](1) == tile(1))){
                flag = 1;
                break;
              }
            }

            if(flag == 0)
              ways_out[ii].tiles.push_back(tile);
            flag = 0;
            //cout<< "size of tiles" << v3dWays[ii].tiles.size()<<endl;
          }
        }
      }

      //set the vector of tags
      for (int tt=0; tt<ways[ii].tags_size(); tt++) {

        ways_out[ii].tags.resize(ways[ii].tags_size());
        ways_out[ii].tags[tt] = ways[ii].tags(tt);
        std::string str = ways_out[ii].tags[tt].key();
        std::transform(str.begin(),str.end(),str.begin(), ::toupper);

        //set the type
        if(!str.compare("BUILDING"))
        {
          ways_out[ii].type = BUILDING;
          break;
        }
        else if (!str.compare("HIGHWAY"))
        {
          ways_out[ii].type = HIGHWAY;
          break;
        }
        else if (!str.compare("RAILWAY"))
        {
          ways_out[ii].type = RAILWAY;
          break;
        }
        else if (!str.compare("AMENITY"))
        {
          ways_out[ii].type = AMENITY;
          break;
        }
        else
          ways_out[ii].type = UNKNOWN;

      }

    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void ParseOpenStreetMapDataNoOrg(int zoom,
                                   const std::vector<node>& nodes,
                                   const std::vector<way>& ways,
                                   const std::vector<relation>& relations,
                                   std::vector<Way>& ways_out
                                   )
  {
    GPSPoint pnt;
    Eigen::Vector3i tile;
    int flag = 0;

    ways_out.resize(ways.size());

    for( size_t ii = 0; ii < ways.size() ; ii++){
      //set ID of the way
      ways_out[ii].wayID = ways[ii].id();
      //set the vector of GPS points
      for (int jj=0; jj < ways[ii].nodes_size(); jj++) {

        for(size_t kk=0; kk< nodes.size(); kk++) {
          if(ways[ii].nodes(jj) == nodes[kk].id()){
            pnt.lat = nodes[kk].lat();
            pnt.lon = nodes[kk].lon();
            CreatePointNoOrg(pnt,zoom);
            ways_out[ii].waypoints.push_back(pnt);
            tile << ways_out[ii].waypoints[jj].tx,ways_out[ii].waypoints[jj].ty,0;

            for(size_t mm = 0; mm < ways_out[ii].tiles.size(); mm++){
              if((ways_out[ii].tiles[mm](0) == tile(0)) && (ways_out[ii].tiles[mm](1) == tile(1))){
                flag = 1;
                break;
              }
            }

            if(flag == 0)
              ways_out[ii].tiles.push_back(tile);
            flag = 0;
          }
        }
      }

      //set the vector of tags
      for(int tt=0; tt<ways[ii].tags_size(); tt++){

        ways_out[ii].tags.resize(ways[ii].tags_size());
        ways_out[ii].tags[tt] = ways[ii].tags(tt);
        std::string str = ways_out[ii].tags[tt].key();
        std::transform(str.begin(),str.end(),str.begin(), ::toupper);

        //set the type
        if(!str.compare("BUILDING"))
        {
          ways_out[ii].type = BUILDING;
          break;
        }
        else if (!str.compare("HIGHWAY"))
        {
          ways_out[ii].type = HIGHWAY;
          break;
        }
        else if (!str.compare("RAILWAY"))
        {
          ways_out[ii].type = RAILWAY;
          break;
        }
        else if (!str.compare("AMENITY"))
        {
          ways_out[ii].type = AMENITY;
          break;
        }
        else
          ways_out[ii].type = UNKNOWN;

      }

    }

  }

  //////////////////////////////////////////////////////////////////////////////
  std::size_t WriteData( char *ptr, std::size_t size, std::size_t nmemb, void *userdata )
  {
    std::vector<char>& bytes = *((std::vector<char>*)userdata);
    bytes.insert( bytes.end(), (char*)ptr, (char*)ptr+size*nmemb );
    return size * nmemb;
  }

  //////////////////////////////////////////////////////////////////////////////

  void FetchTile(const int x,
                 const int y,
                 const int zoom,
                 const int src,
                 std::vector<char>& tile_buffer)
  {
    std::string sBaseURL[3];
    sBaseURL[0] = "http://a.tile.openstreetmap.org/%d/%d/%d.png";
    sBaseURL[1] = "http://oatile1.mqcdn.com/tiles/1.0.0/sat/%d/%d/%d.png";
    sBaseURL[2] = "http://otile3.mqcdn.com/tiles/1.0.0/osm/%d/%d/%d.png";
    //string sTilesSourceName = "mapquest";
    //string sTilesCopyright  = "Map data © OpenStreetMap contributors, CC BY-SA ; tiles courtesy of MapQuest";

    char url[500];
    char filename[500];

    sprintf(url, sBaseURL[src].c_str(), zoom, x, y);
    sprintf(filename, "tile-src%03d-tile-z%03d-x%05d-y%05d.png",src,zoom,x,y);

    CURL *curl;
    CURLcode res;

    std::cout << "downloading: " << url << std::endl;

    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteData);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &tile_buffer);

    if(curl){
      curl_easy_setopt(curl, CURLOPT_URL, url);

      // Grab image
      res = curl_easy_perform(curl);

      if( res != CURLE_OK ) {
        fprintf( stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
      }

      curl_easy_cleanup(curl);
    }

  }

} // namespace osm

