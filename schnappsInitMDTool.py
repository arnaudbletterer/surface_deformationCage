v0 = schnapps.getView("view_0");
c0 = schnapps.getCamera("camera_0");

c0.setProjectionType(1);	#Projection orthographique

importPlugin = schnapps.enablePlugin("Surface_Import");
surfaceRender = schnapps.enablePlugin("Surface_Render");
surfaceSelection = schnapps.enablePlugin("Surface_Selection");
surfaceDeformation = schnapps.enablePlugin("Surface_Deformation");
surfaceGenerationMDTool = schnapps.enablePlugin("Surface_GenerationMDTool");
surfaceDeformationCage = schnapps.enablePlugin("Surface_DeformationCage");

v0.linkPlugin(surfaceRender.getName());
v0.linkPlugin(surfaceSelection.getName());
v0.linkPlugin(surfaceDeformation.getName());
v0.linkPlugin(surfaceGenerationMDTool.getName());
v0.linkPlugin(surfaceDeformationCage.getName());

#mesh = importPlugin.importFromFile("/home/bletterer/Projets/Data/off/bunny.off");

surfaceGenerationMDTool.initializeObject(v0.getName(), 50, 50);
surfaceGenerationMDTool.initializeCages(v0.getName(), 2, 1);

mesh = schnapps.getMap("Model");

v0.linkMap(mesh.getName());

mesh.createVBO("position");

surfaceRender.changePositionVBO(v0.getName(), mesh.getName(), "position");

surfaceRender.changeRenderFaces(v0.getName(), mesh.getName(), False);
surfaceRender.changeRenderEdges(v0.getName(), mesh.getName(), True);
surfaceRender.changeRenderVertices(v0.getName(), mesh.getName(), False);
surfaceRender.changeVerticesScaleFactor(v0.getName(), mesh.getName(), 0.2);

cage = schnapps.getMap("Cages");
v0.linkMap(cage.getName());
cage.createVBO("position");
surfaceRender.changePositionVBO(v0.getName(), cage.getName(), "position");
surfaceRender.changeRenderFaces(v0.getName(), cage.getName(), False);
surfaceRender.changeRenderEdges(v0.getName(), cage.getName(), False);
surfaceRender.changeRenderVertices(v0.getName(), cage.getName(), True);
surfaceRender.changeVerticesScaleFactor(v0.getName(), cage.getName(), 2);
surfaceRender.changeRenderBoundary(v0.getName(), cage.getName(), True);

surfaceDeformationCage.computeAllPointsFromObject(mesh.getName(), cage.getName(), "position", "position");

#v0.unlinkMap(cage.getName());
