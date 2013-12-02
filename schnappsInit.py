v0 = schnapps.getView("view_0");

importPlugin = schnapps.enablePlugin("Surface_Import");
surfaceRender = schnapps.enablePlugin("Surface_Render");
surfaceGenerationCage = schnapps.enablePlugin("Surface_GenerationCage");
surfaceDeformationCage = schnapps.enablePlugin("Surface_DeformationCage");
surfaceDeformation = schnapps.enablePlugin("Surface_Deformation");
surfaceSelection = schnapps.enablePlugin("Surface_Selection");

v0.linkPlugin(surfaceRender.getName());

mesh = importPlugin.importFromFile("/home/abletterer/Projets/Deformation/Data/off/tetra.off");

v0.linkMap(mesh.getName());

mesh.createVBO("position");

surfaceRender.changePositionVBO(v0.getName(), mesh.getName(), "position");

surfaceGenerationCage.generationCage(mesh.getName(), "position");

meshCage = schnapps.getMap(mesh.getName()+'Cage');
v0.linkMap(meshCage.getName());
meshCage.createVBO("position");
surfaceRender.changePositionVBO(v0.getName(), meshCage.getName(), "position");

v0.linkPlugin(surfaceDeformation.getName());
v0.linkPlugin(surfaceSelection.getName());
