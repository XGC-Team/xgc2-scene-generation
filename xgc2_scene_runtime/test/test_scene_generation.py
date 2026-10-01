from pathlib import Path
import json
import sys
import tempfile
import unittest
import yaml
from xgc2_scene_runtime.document import SceneError
from xgc2_scene_runtime.generation import resolve

class GenerationTest(unittest.TestCase):
    def test_external_generator_runs_once_and_saves_exact_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            script = root/'generator.py'
            count = root/'calls'
            script.write_text('import pathlib,json,sys\np=pathlib.Path(sys.argv[1]);p.write_text(p.read_text()+"x" if p.exists() else "x")\nprint(json.dumps({"seed":7,"scene":{"schema":"xgc2.scene.v1","id":"generated","frame":"world","obstacles":[]}}))\n')
            source = root/'source.yaml'
            source.write_text(yaml.safe_dump({'schema':'xgc2.scene-source.v1','mode':'random','format':'geometry','generator':{'command':[sys.executable,str(script),str(count)],'parameters':{'count':60}}}))
            result = root/'run1'/'scene.yaml'
            scene, path = resolve(str(source),str(result),{'count':4})
            self.assertEqual(path,str(result))
            self.assertEqual(scene[0]['id'],'generated')
            self.assertEqual(count.read_text(),'x')
            resolve(str(source),str(result),{'count':9})
            self.assertEqual(count.read_text(),'x')
            provenance=yaml.safe_load(result.with_suffix('.generation.yaml').read_text())
            self.assertEqual(provenance['parameters'],{'count':4})
            self.assertEqual(provenance['seed'],7)
            resolve(str(source),str(root/'run2'/'scene.yaml'))
            self.assertEqual(count.read_text(),'xx')
            with self.assertRaises(SceneError): resolve(str(source),str(root/'run3'/'scene.yaml'),{'typo':1})

    def test_generator_failure_does_not_publish_a_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            source=root/'source.yaml';result=root/'result.yaml'
            source.write_text(yaml.safe_dump({'schema':'xgc2.scene-source.v1','mode':'random','format':'geometry','generator':{'command':[sys.executable,'-c','import sys;sys.exit(3)'],'parameters':{}}}))
            with self.assertRaises(SceneError): resolve(str(source),str(result))
            self.assertFalse(result.exists())

if __name__ == '__main__': unittest.main()
