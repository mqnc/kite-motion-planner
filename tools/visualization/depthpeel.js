
import * as THREE from 'three';
import { FullScreenQuad } from 'pass';

export class DepthPeelingRenderer {

    constructor(parameters) {
        const {
            layers = 5,
            samples = 0,
            colorSpace = THREE.SRGBColorSpace,
            depthBuffer = true,
            ...threeParameters
        } = parameters ?? {};

        this.renderer = new THREE.WebGLRenderer(threeParameters);

        this.numLayers = layers
        this.samples = samples
        this.colorSpace = colorSpace
        this.depthBuffer = depthBuffer
        this.renderTarget = new THREE.WebGLRenderTarget(1, 1, { colorSpace, depthBuffer, samples });
        this.compositeTarget = new THREE.WebGLRenderTarget(1, 1, { colorSpace, depthBuffer, samples });

        this.layers = [];
        this.depthTexture = new THREE.DepthTexture(1, 1, THREE.FloatType);
        this.depthTexture2 = new THREE.DepthTexture(1, 1, THREE.FloatType);
        this.opaqueDepthTexture = new THREE.DepthTexture(1, 1, THREE.FloatType);
        this.copyQuad = new FullScreenQuad(new THREE.MeshBasicMaterial());

        // We can't inherit from WebGLRenderer since it assigns all its methods in its constructor
        // so we can't override them. Instead we favor composition over inheritance
        // and forward everything that is not overridden to the renderer member.
        return new Proxy(this, {
            get: (target, prop, receiver) => {
                if (prop in target) return Reflect.get(target, prop, receiver);
                const value = target.renderer[prop];
                return typeof value === 'function' ? value.bind(target.renderer) : value;
            },
            set: (target, prop, value) => {
                if (prop in target) target[prop] = value;
                else target.renderer[prop] = value;
                return true;
            }
        });
    }

    setPixelRatio(value) {
        this.renderer.setPixelRatio(value)
        this._resize()
    }

    setSize(width, height, updateStyle) {
        this.renderer.setSize(width, height, updateStyle)
        this._resize()
    }

    _resize() {
        const [w, h] = this.renderer.getSize(new THREE.Vector2()).toArray()
        const dpr = this.renderer.getPixelRatio()

        this.compositeTarget.setSize(dpr * w, dpr * h);
        this.renderTarget.setSize(dpr * w, dpr * h);

        this.layers.forEach(rt => rt.dispose());
        this.layers.length = 0;

        this.depthTexture.image.width = dpr * w;
        this.depthTexture.image.height = dpr * h;
        this.depthTexture.dispose();

        this.depthTexture2.image.width = dpr * w;
        this.depthTexture2.image.height = dpr * h;
        this.depthTexture2.dispose();

        this.opaqueDepthTexture.image.width = dpr * w;
        this.opaqueDepthTexture.image.height = dpr * h;
        this.opaqueDepthTexture.dispose();
    }

    render(scene, camera) {
        const [w, h] = this.renderer.getSize(new THREE.Vector2()).toArray()
        const dpr = this.renderer.getPixelRatio()

        const originalTarget = this.renderer.getRenderTarget()

        while (this.layers.length < this.numLayers) {
            this.layers.push(new THREE.WebGLRenderTarget(w * dpr, h * dpr, {
                colorSpace: this.colorSpace,
                depthBuffer: this.depthBuffer,
                samples: this.samples,
            }));
        }

        while (this.layers.length > this.numLayers) {
            this.layers.pop().dispose();
        }

        let opaqueMaterials = new Set()
        let transparentMaterials = new Set()

        scene.traverse(({ material }) => {
            if (material && material.isDepthPeelMaterial) {
                if (material.transparent) { transparentMaterials.add(material) }
                else { opaqueMaterials.add(material) }
            }
        })

        // render opaque layer
        for (let material of opaqueMaterials) { material.visible = true }
        for (let material of transparentMaterials) { material.visible = false }
        this.renderTarget.depthTexture = this.opaqueDepthTexture;
        this.renderer.setRenderTarget(this.renderTarget);
        this.renderer.render(scene, camera);
        this.renderer.setRenderTarget(null);

        this.copyQuad.material.map = this.renderTarget.texture;
        this.copyQuad.material.blending = THREE.NoBlending;
        this.copyQuad.material.transparent = false;
        this.copyQuad.material.depthTest = false;
        this.copyQuad.material.depthWrite = false;
        this.copyQuad.render({ render: this.renderer.render.bind(this.renderer) });
        this.renderTarget.depthTexture = null;

        const clearAlpha = this.renderer.getClearAlpha();
        const clearColor = this.renderer.getClearColor(new THREE.Color());

        // perform depth peeling
        for (let material of opaqueMaterials) { material.visible = false }
        for (let material of transparentMaterials) {
            material.visible = true
            material.enableDepthPeeling = true;
            material.opaqueDepth = this.opaqueDepthTexture;
            material.blending = THREE.CustomBlending;
            material.blendDst = THREE.ZeroFactor;
            material.blendSrc = THREE.OneFactor;
            material.depthWrite = true;
            material.forceSinglePass = true;
            this.renderer.getDrawingBufferSize(material.resolution);
        }
        for (let i = 0; i < this.numLayers; i++) {

            const depthTextures = [this.depthTexture, this.depthTexture2];
            const writeDepthTexture = depthTextures[(i + 1) % 2];
            const nearDepthTexture = depthTextures[i % 2];

            // update the materials, skipping the near check
            for (let material of transparentMaterials) {
                material.nearDepth = i === 0 ? null : nearDepthTexture;
            }

            // perform rendering
            let currTarget = i === 0 ? this.compositeTarget : this.renderTarget;
            currTarget = this.layers[i];
            currTarget.depthTexture = writeDepthTexture;

            this.renderer.setRenderTarget(currTarget);
            this.renderer.setClearColor(0, 0);
            this.renderer.render(scene, camera);
            this.renderer.setRenderTarget(null);
        }

        this.renderer.setRenderTarget(originalTarget);
        this.renderer.setClearColor(clearColor, clearAlpha);

        // render transparent layers
        for (let i = this.numLayers - 1; i >= 0; i--) {

            this.autoClear = false;
            this.layers[i].depthTexture = null;
            this.copyQuad.material.map = this.layers[i].texture;
            this.copyQuad.material.blending = THREE.NormalBlending;
            this.copyQuad.material.transparent = true;
            this.copyQuad.material.depthTest = false;
            this.copyQuad.material.depthWrite = false;
            this.copyQuad.render({ render: this.renderer.render.bind(this.renderer) });

        }

        this.renderer.autoClear = true;

    }
}

export function DepthPeelMaterialMixin(baseMaterial) {
    return class extends baseMaterial {
        isDepthPeelMaterial = true

        get nearDepth() {
            return this._uniforms.nearDepth.value;
        }

        set nearDepth(v) {
            this._uniforms.nearDepth.value = v;
            this.needsUpdate = true;
        }

        get opaqueDepth() {
            return this._uniforms.opaqueDepth.value;
        }

        set opaqueDepth(v) {
            this._uniforms.opaqueDepth.value = v;
        }

        get enableDepthPeeling() {
            return this._enableDepthPeeling;
        }

        set enableDepthPeeling(v) {
            if (this._enableDepthPeeling !== v) {
                this._enableDepthPeeling = v;
                this.needsUpdate = true;
            }
        }

        get resolution() {
            return this._uniforms.resolution.value;
        }

        constructor(...args) {
            super(...args);
            this._firstPass = false;
            this._enableDepthPeeling = false;
            this._uniforms = {
                nearDepth: { value: null },
                opaqueDepth: { value: null },
                resolution: { value: new THREE.Vector2() },
            };
        }

        customProgramCacheKey() {
            return `${Number(this.enableDepthPeeling)}|${Number(this.nearDepth)}`;
        }

        onBeforeCompile(shader) {
            shader.uniforms = {
                ...shader.uniforms,
                ...this._uniforms,
            };
            shader.fragmentShader =
                /* glsl */`
                    #define DEPTH_PEELING ${Number(this.enableDepthPeeling)}
                    #define FIRST_PASS ${Number(!this.nearDepth)}
                    
                    #if DEPTH_PEELING
                    
                    uniform sampler2D nearDepth;
                    uniform sampler2D opaqueDepth;
                    uniform vec2 resolution;

                    #endif

                    ${shader.fragmentShader}
                `.replace('void main() {', /* glsl */`
                    void main() {
                        #if DEPTH_PEELING
                        vec2 screenUV = gl_FragCoord.xy / resolution;
                        if ( texture2D( opaqueDepth, screenUV ).r < gl_FragCoord.z ) {
                            discard;
                        }
                        #if ! FIRST_PASS
                        if ( texture2D( nearDepth, screenUV ).r >= gl_FragCoord.z - 1e-6 ) {
                            discard;
                        }
                        #endif
                        #endif
                ` );
        }
    };
}
