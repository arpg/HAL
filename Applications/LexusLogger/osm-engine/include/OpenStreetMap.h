
#pragma once

#include <stdio.h>
#include <curl/curl.h>
#include <vector>
#include <ctype.h>
#include <eigen3/Eigen/Core>
#include <osm.pb.h>
#include <TinyXML.h>

namespace osm
{

  typedef struct
  {
    double lat;     // latitude in degrees [-180 180]
    double lon;     // longitude in degrees [-180 180]
    double xpos;    // x position inside the tile [0 1]
    double ypos;    // y position inside the tile [0 1]
    int    tx;      // x tile index
    int    ty;      // y tile index
    double x;       // x metric
    double y;       // y metric
    double z;       // z metric
    bool   active;  // true if GPSPoint is in the current tile

  } GPSPoint;

  typedef std::vector<GPSPoint> WayPoints;
  typedef std::vector<osm::tag> WayTags;
  typedef std::vector<Eigen::Vector3i> WayTiles;
  enum WayType { BUILDING=0, HIGHWAY=1, RAILWAY=2, AMENITY=3, UNKNOWN=4 };


  class Way{
  public:
    int wayID;
    WayPoints waypoints;
    WayTags tags;
    WayType type;
    WayTiles tiles;
  };


  double LatitudeToTileY(double lat, int zoom);

  double LongitudeToTileX(double lon, int zoom);

  double TileYToLatitude(int y, int zoom);

  double TileXToLongitude(int x, int zoom);

  // Compute gps point tile coordinate and relative position
  void CreatePoint(GPSPoint &pnt, int zoom, GPSPoint origin, float resolution);

  // Compute gps point tile coordinate and relative position
  void CreatePointNoOrg(GPSPoint &pnt, int zoom);

  size_t my_dummy_write(char *ptr, size_t size, size_t nmemb, void *userdata);

  template<typename T>
  T GetAttrib(rpg::TiXmlNode* xml_node, const std::string attr );

  void ParseNode(rpg::TiXmlNode* xml_node, osm::node& n);

  void ParseWay(rpg::TiXmlNode* way_node, osm::way& way_pb);

  void ParseRelation(rpg::TiXmlNode* relation_node, osm::relation& relation_pb);

  ///
  /// \brief FetchOpenStreetMapData
  /// \param[in] url
  /// \param[out] nodes
  /// \param[out] ways
  /// \param[out] relations
  ///
  void FetchOpenStreetMapData(
      const std::string           url,
      std::vector<osm::node>&     nodes,
      std::vector<osm::way>&      ways,
      std::vector<osm::relation>& relations
      );

  ///
  /// \brief ParseOpenStreetMapData
  /// \param[in] origin
  /// \param[in] resolution
  /// \param[in] zoom
  /// \param[in] nodes
  /// \param[in] ways
  /// \param[in] relations
  /// \param[out] ways_out
  ///
  void ParseOpenStreetMapData(
      GPSPoint                          origin,
      float                             resolution,
      int                               zoom,
      const std::vector<osm::node>&     nodes,
      const std::vector<osm::way>&      ways,
      const std::vector<osm::relation>& relations,
      std::vector<Way>&                 ways_out
      );

  ///
  /// \brief ParseOpenStreetMapDataNoOrg
  /// \param[in] zoom
  /// \param[in] nodes
  /// \param[in] ways
  /// \param[in] relations
  /// \param[out] ways_out
  ///
  void ParseOpenStreetMapDataNoOrg(
      int                               zoom,
      const std::vector<osm::node>&     nodes,
      const std::vector<osm::way>&      ways,
      const std::vector<osm::relation>& relations,
      std::vector<Way>&                 ways_out
      );


  ///
  /// \brief FetchTile
  /// \param[in] x
  /// \param[in] y
  /// \param[in] zoom
  /// \param[in] src
  /// \param[out] tile_buffer
  ///
  void FetchTile( const int x,
                  const int y,
                  const int zoom,
                  const int src,
                  std::vector<char>& tile_buffer);

  ///
  /// \brief WriteData
  /// \param ptr
  /// \param size
  /// \param nmemb
  /// \param userdata
  /// \return
  ///
  std::size_t WriteData( char *ptr, std::size_t size,
                         std::size_t nmemb, void *userdata );


} // namespace osm

