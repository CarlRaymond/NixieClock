from xml.etree.ElementTree import ElementTree;
from copy import deepcopy;

tree = ElementTree();

ns = {
'svg': "http://www.w3.org/2000/svg",
'inkscape': "http://www.inkscape.org/namespaces/inkscape",
'sodipodi': "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
}

#for attr, value in ns.iteritems():
#	tree.register_namespace(attr, value)

def generate(filaments, template, folder):

	tree.parse(template);
	root = tree.getroot();


	# Find layer _ACTIVE_, keep a reference, and remove from document
	active = tree.find("svg:g[@inkscape:label='_ACTIVE_']", ns);
	if active is None:
		print("No '_ACTIVE_' layer found.");
		exit();
	root.remove(active);

	# Find layer _INACTIVE_, keep a reference, and remove from document
	inactive = tree.find("svg:g[@inkscape:label='_INACTIVE_']", ns);
	if inactive is None:
		print("No '_INACTIVE_' layer found.");
		exit();
	root.remove(inactive);

	activeLayers = [];
	inactiveLayers = [];
	insertPos = 4;

	# Create active and inactive version of every layer.
	# Ensure layer is visible
	for idx, fil in enumerate(filaments):
		layer = deepcopy(active);
		layer.set("{" + ns["inkscape"] + "}label", fil);
		layer.set("style", "display:inline");
		tspan = layer.find("svg:text/svg:tspan", ns);
		tspan.text = fil
		activeLayers.append(layer);

		layer = deepcopy(inactive);
		layer.set("{" + ns["inkscape"] + "}label", fil);
		layer.find("svg:text/svg:tspan", ns).text = fil;
		inactiveLayers.append(layer);

	# For each filament, combine inactive and active versions, insert into
	# the document, save it, then remove the layers from the document
	for idx, fil in enumerate(filaments):
		layers = inactiveLayers[:idx] + [activeLayers[idx]] + inactiveLayers[idx+1:]
		for l in layers:
			root.insert(insertPos, l)
		tree.write(folder + "/" + fil + ".svg");

		# Remove layers
		for l in layers:
			root.remove(l)

	# Blank all filaments
	for l in inactiveLayers:
		root.insert(insertPos, l)
	tree.write(folder + "/blank.svg");
	return

generate(["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"], "TLA_template.svg", "months");
generate([ "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" ], "TLA_template.svg", "days");
#filaments = [ "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY" ]
#filaments = [ "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" ];

