from xml.etree.ElementTree import ElementTree;
from copy import deepcopy;

ns = {
'svg': "http://www.w3.org/2000/svg",
'inkscape': "http://www.inkscape.org/namespaces/inkscape",
'sodipodi': "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
}

folder = "pairs";

tree = ElementTree();
tree.parse("digit_template.svg");
root = tree.getroot();

# Find layer _ACTIVE_UNITS_, keep a reference, and remove from document
activeUnits = tree.find("svg:g[@inkscape:label='_ACTIVE_UNITS_']", ns);
if activeUnits is None:
	print("No '_ACTIVE_UNITS_' layer found.");
	exit();
root.remove(activeUnits);

# Find layer _ACTIVE_TENS_, keep a reference, and remove from document
activeTens = tree.find("svg:g[@inkscape:label='_ACTIVE_TENS_']", ns);
if activeTens is None:
	print("No '_ACTIVE_TENS_' layer found.");
	exit();
root.remove(activeTens);

# Find layer _INACTIVE_UNITS_, keep a reference, and remove from document
inactiveUnits = tree.find("svg:g[@inkscape:label='_INACTIVE_UNITS_']", ns);
if inactiveUnits is None:
	print("No '_INACTIVE_UNITS_' layer found.");
	exit();
root.remove(inactiveUnits);

# Find layer _INACTIVE_TENSS_, keep a reference, and remove from document
inactiveTens = tree.find("svg:g[@inkscape:label='_INACTIVE_TENS_']", ns);
if inactiveTens is None:
	print("No '_INACTIVE_TENS_' layer found.");
	exit();
root.remove(inactiveTens);

activeUnitsLayers = [];
activeTensLayers = [];
inactiveUnitsLayers = [];
inactiveTensLayers = [];
insertPos = 4;

# Create active and inactive version of every layer.
# Ensure layer is visible
for idx, fil in enumerate(["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", " "]):
	layer = deepcopy(activeUnits);
	layer.set("{" + ns["inkscape"] + "}label", fil);
	layer.set("style", "display:inline");
	tspan = layer.find("svg:text/svg:tspan", ns);
	tspan.text = fil
	activeUnitsLayers.append(layer);

	layer = deepcopy(inactiveUnits);
	layer.set("{" + ns["inkscape"] + "}label", fil);
	layer.find("svg:text/svg:tspan", ns).text = fil;
	inactiveUnitsLayers.append(layer);

for idx, fil in enumerate(["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", " "]):
	layer = deepcopy(activeTens);
	layer.set("{" + ns["inkscape"] + "}label", fil);
	layer.set("style", "display:inline");
	tspan = layer.find("svg:text/svg:tspan", ns);
	tspan.text = fil
	activeTensLayers.append(layer);

	layer = deepcopy(inactiveTens);
	layer.set("{" + ns["inkscape"] + "}label", fil);
	layer.find("svg:text/svg:tspan", ns).text = fil;
	inactiveTensLayers.append(layer);

def createPair(tensIdx, unitsIdx, basename):

	# Combine layers for indicated combinations
	tensLayers = inactiveTensLayers[:tensIdx] + [activeTensLayers[tensIdx]] + inactiveTensLayers[tensIdx+1:]
	unitsLayers = inactiveUnitsLayers[:unitsIdx] + [activeUnitsLayers[unitsIdx]] + inactiveUnitsLayers[unitsIdx+1:]

	for l in tensLayers + unitsLayers:
		root.insert(insertPos, l)
	
	# Write file
	tree.write(folder + "/" + basename + ".svg");

	# Remove digit layers
	for l in tensLayers + unitsLayers:
		root.remove(l);
	return

for u in range(0, 10):
	createPair(10, u, f"{u}");

for t in range(0, 10):
	for u in range(0, 10):
		createPair(t, u, f"{t}{u}");

createPair(10, 10, "blank");

