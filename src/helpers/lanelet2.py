from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from shapely.geometry import LineString, Polygon
import numpy as np


def load_lanelet2_map(lanelet2_map_name, coordinate_transformer, use_custom_origin, utm_origin_lat, utm_origin_lon):
    """
    Load a lanelet2 map from a file and return it
    :param lanelet2_map_name: name of the lanelet2 map file
    :param coordinate_transformer: coordinate transformer
    :param use_custom_origin: use custom origin
    :param utm_origin_lat: utm origin latitude
    :param utm_origin_lon: utm origin longitude
    :return: lanelet2 map
    """

    # Load the map using Lanelet2
    if coordinate_transformer == "utm":
        projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
    else:
        raise ValueError('Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + coordinate_transformer)

    lanelet2_map = load(lanelet2_map_name, projector)

    return lanelet2_map

def get_crosswalks(lanelet2_map):
    """
    Find all crosswalks on map and return a dictionary with all linked stopline ids and a polygon
    :param lanelet2_map: lanelet2 map
    :return: {lanelet_id: {stopline_ids: [stopline_ids], polygon: Polygon}}
    """

    crosswalks = {}
    for lanelet in lanelet2_map.laneletLayer:
        if lanelet.attributes:
            if lanelet.attributes["subtype"] == "crosswalk":
                # add crosswalk to dictionary
                stopline_ids = []
                if "stopline_ids" in lanelet.attributes:
                    stopline_ids = lanelet.attributes["stopline_ids"]
                    stopline_ids = [int(x) for x in stopline_ids.split(",")]
                crosswalks[lanelet.id] = {"stopline_ids": stopline_ids, "polygon": Polygon([(p.x, p.y) for p in lanelet.polygon2d()])}

    return crosswalks


def get_stoplines(lanelet2_map):
    """
    Add all stop lines to a dictionary with stop_line id as key and stop_line as value
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: stopline, ...}
    """

    stoplines = {}
    for line in lanelet2_map.lineStringLayer:
        if line.attributes:
            if line.attributes["type"] == "stop_line":
                # add stoline to dictionary and convert it to shapely LineString
                stoplines[line.id] = LineString([(p.x, p.y) for p in line])

    return stoplines


def get_stoplines_api_id(lanelet2_map):
    """
    Iterate over all stop lines and extract all stop lines that have api_id and add to dict
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: stop_line.api_id, ...}
    """

    # extract all stop lines that have api_id and add to dict
    stopline_ids = {}
    for line in lanelet2_map.lineStringLayer:
        if line.attributes:
            if line.attributes["type"] == "stop_line":
                if "api_id" in line.attributes:
                    stopline_ids[line.id] = line.attributes["api_id"]

    return stopline_ids


def get_stoplines_trafficlights_bulbs(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stoplines and sinals.
    Organize the data into dictionary indexed by stopline id that contains a traffic_light id and light bulb data.
    :param lanelet2_map: lanelet2 map
    :return: {stopline_id: {traffic_light_id: [[bulb_id, bulb_color, x, y, z], ...], ...}, ...}
    """

    signals = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
            linkId = reg_el.parameters["ref_line"][0].id

            for bulbs in reg_el.parameters["light_bulbs"]:
                # plId represents the traffic light (pole), one stop line can be associated with multiple traffic lights
                plId = bulbs.id
                # one traffic light has red, yellow and green bulbs
                bulb_data = [[bulb.id, bulb.attributes["color"], bulb.x, bulb.y, bulb.z] for bulb in bulbs]
                # signals is a dictionary indexed by stopline id and contains dictionary of traffic lights indexed by pole id
                # which in turn contains a list of bulbs
                signals.setdefault(linkId, {}).setdefault(plId, []).extend(bulb_data)

    return signals

def get_stoplines_center(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stoplines centers.
    Organize the data into dictionary indexed by stopline id that contains stopline center coordinates and respective traffic light ids
    :param lanelet2_map: lanelet2 map
    :return: {stopline_id: [[(center_x, center_y), [PlIds]], ...], ...}
    """

    stopline_centers = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
            link = reg_el.parameters["ref_line"][0]
            # Get all geometry points of the stopline
            line_points = [[point.x, point.y] for point in link]
            # Extract center point from stopline
            center_x, center_y = np.mean(line_points, axis=0)
            # Extract traffic light (Pole) ids for the same stopline
            plIds = [bulbs.id for bulbs in reg_el.parameters["light_bulbs"]]

            stopline_centers[link.id] = [(center_x, center_y), plIds] 

    return stopline_centers