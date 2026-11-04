import * as THREE from 'three';

// Height is map Z, independent of camera orientation. RGB buffers remain intact.
export function installHeightColors(material) {
 const uniforms = {
  heightEnabled: {value: false},
  heightRange: {value: new THREE.Vector2(0, 1)},
  heightPalette: {value: ['#3264e8', '#18bce1', '#54ce91', '#f3db57', '#f07836'].map(hex => new THREE.Color(hex))},
 };
 material.onBeforeCompile = shader => {
  Object.assign(shader.uniforms, uniforms);
  shader.vertexShader = 'varying float mapHeight;\n' + shader.vertexShader.replace(
   '#include <begin_vertex>', '#include <begin_vertex>\nmapHeight = position.z;');
  shader.fragmentShader = `
   varying float mapHeight;
   uniform bool heightEnabled;
   uniform vec2 heightRange;
   uniform vec3 heightPalette[5];
   vec3 heightColor(float height) {
    float t = clamp((height-heightRange.x)/max(heightRange.y-heightRange.x,0.001),0.0,1.0)*4.0;
    if(t<1.0) return mix(heightPalette[0],heightPalette[1],t);
    if(t<2.0) return mix(heightPalette[1],heightPalette[2],t-1.0);
    if(t<3.0) return mix(heightPalette[2],heightPalette[3],t-2.0);
    return mix(heightPalette[3],heightPalette[4],t-3.0);
   }
  ` + shader.fragmentShader.replace('#include <color_fragment>',
   '#include <color_fragment>\nif(heightEnabled) diffuseColor.rgb = heightColor(mapHeight);');
 };
 material.customProgramCacheKey = () => 'liujing-height-v1';
 return uniforms;
}
