import unittest
import xml.etree.ElementTree as ET
import shape


def sample():
    return '''<sdf version="1.9"><model name="eval_frame"><static>true</static><pose>1 2 .07 0 0 .3</pose>
    <link name="body"><collision name="collision"/><visual name="visual"/>
    <sensor name="eval_contact"><contact><topic>/eval/contact</topic></contact></sensor></link>
    <plugin name="pose" filename="gz-sim-pose-publisher-system"/></model></sdf>'''


ACTOR=dict(model_name='eval_frame',length_m=.20,width_m=.15,height_m=.14,bottom_m=0.)


class ShapeTests(unittest.TestCase):
    def test_beam_sees_four_posts_and_not_base_or_roof(self):
        self.assertEqual(shape.visible_parts(.095),['post_fl','post_fr','post_rl','post_rr'])
        self.assertEqual(shape.visible_parts(.01),['base'])
        self.assertEqual(shape.visible_parts(.13),['roof'])

    def test_preserves_pose_and_covered_evaluation_footprint(self):
        text=shape.replace_box(sample(),ACTOR)
        root=ET.fromstring(text)
        self.assertEqual(root.findtext('model/pose'),'1 2 .07 0 0 .3')
        self.assertIsNotNone(root.find('model/plugin'))
        self.assertTrue(shape.audit(text,ACTOR)['passed'])
        for _,xyz,size in shape.PARTS:
            self.assertLessEqual(abs(xyz[0])+size[0]/2,.10000001)
            self.assertLessEqual(abs(xyz[1])+size[1]/2,.07500001)

    def test_wrong_dimensions_rejected(self):
        for key in ('length_m','width_m','height_m','bottom_m'):
            with self.assertRaises(ValueError):shape.replace_box(sample(),{**ACTOR,key:.3})

    def test_each_part_has_contact_sensor(self):
        root=ET.fromstring(shape.replace_box(sample(),ACTOR))
        sensors=root.findall('model/link/sensor')
        self.assertEqual(len(sensors),6)
        self.assertEqual({s.findtext('contact/topic') for s in sensors},{'/eval/contact'})
        self.assertEqual({s.findtext('contact/collision') for s in sensors},{n for n,_,_ in shape.PARTS})

    def test_audit_rejects_changed_visible_geometry(self):
        root=ET.fromstring(shape.replace_box(sample(),ACTOR))
        root.find("model/link/visual[@name='post_fl']/geometry/box/size").text='.03 .02 .10'
        self.assertFalse(shape.audit(ET.tostring(root,encoding='unicode'),ACTOR)['passed'])


if __name__=='__main__':unittest.main()
